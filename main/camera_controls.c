#include "camera_controls.h"
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <sys/ioctl.h>
#include <sys/lock.h>
#include "esp_log.h"
#include "esp_cam_sensor.h"
#include "esp_ipa_types.h"
#include "esp_video_device.h"
#include "esp_video_ioctl.h"
#include "esp_video_isp_ioctl.h"
#include "linux/videodev2.h"

static const char *TAG = "uvc_controls";
static int s_camera = -1, s_isp = -1;
/* Also held by the IPA hook: no automatic write can race a manual change.
 * Never call an esp_video_isp_pipeline_* API while holding this lock. */
static _lock_t s_lock;
static uint32_t s_manual_mask;
static uint32_t s_tline_ns;
static bool s_auto_exposure = true, s_auto_wb = true, s_reset_ae;
static int32_t s_exp_min, s_exp_max, s_exp_step;
static esp_video_isp_sharpen_t s_sharp_base;
static esp_video_isp_gamma_ext_t s_gamma_base;
static uint16_t s_sharp = 100, s_gamma = 100;
#define WB_MASK (IPA_METADATA_FLAGS_RG | IPA_METADATA_FLAGS_BG)
#define AE_MASK (IPA_METADATA_FLAGS_ET | IPA_METADATA_FLAGS_GN)
#define C(e,s,b,n,sg,lo,hi,step,d,id_,mask) \
    {e,s,b,n,sg,false,lo,hi,step,d,id_,mask}
static icg_control_t s_controls[] = {
    C(ICG_CT,CT_AE_MODE,1,1,false,1,2,3,2,0,0),
    C(ICG_CT,CT_EXPOSURE,3,4,false,1,1,1,1,V4L2_CID_EXPOSURE,0),
    C(ICG_CT,CT_FOCUS,5,2,false,509,605,1,552,V4L2_CID_FOCUS_ABSOLUTE,0),
    C(ICG_PU,PU_BRIGHTNESS,0,2,true,0,0,1,0,V4L2_CID_BRIGHTNESS,IPA_METADATA_FLAGS_BR),
    C(ICG_PU,PU_CONTRAST,1,2,false,0,0,1,0,V4L2_CID_CONTRAST,IPA_METADATA_FLAGS_CN),
    C(ICG_PU,PU_HUE,2,2,true,0,0,1,0,V4L2_CID_HUE,IPA_METADATA_FLAGS_HUE),
    C(ICG_PU,PU_SATURATION,3,2,false,0,0,1,0,V4L2_CID_SATURATION,IPA_METADATA_FLAGS_ST),
    C(ICG_PU,PU_SHARPNESS,4,2,false,0,200,1,100,0,IPA_METADATA_FLAGS_SH),
    C(ICG_PU,PU_GAMMA,5,2,false,50,300,1,100,0,IPA_METADATA_FLAGS_GAMMA),
    C(ICG_PU,PU_WB_COMPONENT,7,4,false,1,3999,1,1000,0,WB_MASK),
    C(ICG_PU,PU_GAIN,9,2,false,0,0,1,0,V4L2_CID_GAIN,0),
    C(ICG_PU,PU_WB_AUTO,13,1,false,0,1,1,1,0,0),
};

/* Called only by the generated esp_video pipeline hook. */
uint32_t icg_ipa_controls_begin(void)
{
    _lock_acquire(&s_lock);
    return s_manual_mask;
}
bool icg_ipa_controls_reset_ae(void)
{
    bool reset = s_reset_ae;
    s_reset_ae = false;
    return reset;
}
void icg_ipa_controls_end(void) { _lock_release(&s_lock); }

static bool scalar(int fd, uint32_t id, int32_t *value, bool write)
{
    struct v4l2_ext_control c = {.id = id, .value = *value};
    struct v4l2_ext_controls cs = {
        .ctrl_class = (id == V4L2_CID_EXPOSURE || id == V4L2_CID_FOCUS_ABSOLUTE)
                      ? V4L2_CID_CAMERA_CLASS : V4L2_CID_USER_CLASS,
        .count = 1, .controls = &c,
    };
    if (ioctl(fd, write ? VIDIOC_S_EXT_CTRLS : VIDIOC_G_EXT_CTRLS, &cs)) return false;
    *value = c.value;
    return true;
}
static bool blob(uint32_t id, void *p, uint32_t size, bool write)
{
    struct v4l2_ext_control c = {.id = id, .p_u8 = p, .size = size};
    struct v4l2_ext_controls cs = {
        .ctrl_class = V4L2_CID_USER_CLASS, .count = 1, .controls = &c,
    };
    return ioctl(s_isp, write ? VIDIOC_S_EXT_CTRLS : VIDIOC_G_EXT_CTRLS, &cs) == 0;
}
static int ctrl_fd(const icg_control_t *c)
{
    return c->entity == ICG_CT || c->selector == PU_GAIN ? s_camera : s_isp;
}
const icg_control_t *camera_control_find(uint8_t entity, uint8_t selector)
{
    for (unsigned i = 0; i < sizeof(s_controls)/sizeof(s_controls[0]); ++i) {
        if (s_controls[i].enabled && s_controls[i].entity == entity &&
            s_controls[i].selector == selector) return &s_controls[i];
    }
    return NULL;
}
uint32_t camera_control_bitmap(uint8_t entity)
{
    uint32_t bits = 0;
    for (unsigned i = 0; i < sizeof(s_controls)/sizeof(s_controls[0]); ++i)
        if (s_controls[i].enabled && s_controls[i].entity == entity)
            bits |= 1u << s_controls[i].bit;
    return bits;
}
static bool automatic(const icg_control_t *c)
{
    return ((c->entity == ICG_CT && c->selector == CT_EXPOSURE) ||
            (c->entity == ICG_PU && c->selector == PU_GAIN)) ? s_auto_exposure :
           (c->entity == ICG_PU && c->selector == PU_WB_COMPONENT && s_auto_wb);
}
uint8_t camera_control_info(const icg_control_t *c)
{
    _lock_acquire(&s_lock);
    uint8_t info = 3 | (automatic(c) ? 0x0c : 0); /* GET/SET, disabled by auto, auto-update */
    _lock_release(&s_lock);
    return info;
}
int camera_control_get(const icg_control_t *c, uint32_t *value)
{
    bool ok = true;
    int32_t v = 0;
    _lock_acquire(&s_lock);
    if (c->entity == ICG_CT && c->selector == CT_AE_MODE) v = s_auto_exposure ? 2 : 1;
    else if (c->entity == ICG_PU && c->selector == PU_WB_AUTO) v = s_auto_wb;
    else if (c->entity == ICG_PU && c->selector == PU_SHARPNESS) v = s_sharp;
    else if (c->entity == ICG_PU && c->selector == PU_GAMMA) v = s_gamma;
    else if (c->entity == ICG_PU && c->selector == PU_WB_COMPONENT) {
        esp_video_isp_wb_t wb;
        ok = blob(V4L2_CID_USER_ESP_ISP_WB, &wb, sizeof(wb), false);
        if (ok) v = (uint32_t)lroundf(wb.blue_gain * 1000) |
                     ((uint32_t)lroundf(wb.red_gain * 1000) << 16);
    } else {
        ok = scalar(ctrl_fd(c), c->id, &v, false);
        if (ok && c->entity == ICG_CT && c->selector == CT_EXPOSURE)
            v = ((uint64_t)v * s_tline_ns + 50000) / 100000;
        if (ok && c->entity == ICG_PU && c->selector == PU_HUE && v > 180) v -= 360;
    }
    *value = (uint32_t)v;
    _lock_release(&s_lock);
    return ok ? CTRL_OK : CTRL_NOT_READY;
}
int camera_control_set(const icg_control_t *c, uint32_t value)
{
    int32_t v = c->signed_value ? (int16_t)value : (int32_t)value;
    bool wb = c->entity == ICG_PU && c->selector == PU_WB_COMPONENT;
    if (wb) {
        if ((value & 65535) < 1 || (value & 65535) > 3999 ||
            (value >> 16) < 1 || (value >> 16) > 3999) return CTRL_RANGE;
    } else if (v < c->min || v > c->max ||
               (c->res > 1 && c->selector != CT_AE_MODE && (v-c->min)%c->res)) return CTRL_RANGE;
    bool ok = true;
    _lock_acquire(&s_lock);
    if (automatic(c)) { _lock_release(&s_lock); return CTRL_WRONG_STATE; }
    if (c->entity == ICG_CT && c->selector == CT_AE_MODE) {
        s_auto_exposure = v == 2;
        if (s_auto_exposure) { s_manual_mask &= ~AE_MASK; s_reset_ae = true; }
        else s_manual_mask |= AE_MASK;
    } else if (c->entity == ICG_PU && c->selector == PU_WB_AUTO) {
        s_auto_wb = v != 0;
        if (s_auto_wb) s_manual_mask &= ~WB_MASK;
        else s_manual_mask |= WB_MASK;
    } else if (wb) {
        esp_video_isp_wb_t balance = {.enable = true,
            .red_gain = (value >> 16) / 1000.0f, .blue_gain = (value & 65535) / 1000.0f};
        ok = blob(V4L2_CID_USER_ESP_ISP_WB, &balance, sizeof(balance), true);
    } else if (c->entity == ICG_PU && c->selector == PU_SHARPNESS) {
        esp_video_isp_sharpen_t sharp = s_sharp_base;
        sharp.h_coeff *= v / 100.0f;
        sharp.m_coeff *= v / 100.0f;
        sharp.enable = v != 0;
        ok = blob(V4L2_CID_USER_ESP_ISP_SHARPEN, &sharp, sizeof(sharp), true);
        if (ok) s_sharp = v;
    } else if (c->entity == ICG_PU && c->selector == PU_GAMMA) {
        /* 100 preserves the tuned per-channel curves; other values adjust them.
         * Keep X coordinates to satisfy hardware power-of-two segment lengths. */
        esp_video_isp_gamma_ext_t gamma = s_gamma_base;
        gamma.flags = ESP_VIDEO_ISP_GAMMA_EXT_FLAG_RED |
                      ESP_VIDEO_ISP_GAMMA_EXT_FLAG_GREEN | ESP_VIDEO_ISP_GAMMA_EXT_FLAG_BLUE;
        for (int i = 0; i < ISP_GAMMA_CURVE_POINTS_NUM; ++i) {
            gamma.red_points[i].y = lroundf(255 * powf(s_gamma_base.red_points[i].y / 255.0f, 100.0f/v));
            gamma.green_points[i].y = lroundf(255 * powf(s_gamma_base.green_points[i].y / 255.0f, 100.0f/v));
            gamma.blue_points[i].y = lroundf(255 * powf(s_gamma_base.blue_points[i].y / 255.0f, 100.0f/v));
        }
        gamma.enable = true;
        ok = blob(V4L2_CID_USER_ESP_ISP_GAMMA_EXT, &gamma, sizeof(gamma), true);
        if (ok) s_gamma = v;
    } else {
        if (c->entity == ICG_CT && c->selector == CT_EXPOSURE) {
            v = ((uint64_t)value * 100000 + s_tline_ns/2) / s_tline_ns;
            v = s_exp_min + ((v-s_exp_min)/s_exp_step)*s_exp_step;
            if (v < s_exp_min) v = s_exp_min;
            if (v > s_exp_max) v = s_exp_max;
        }
        if (c->entity == ICG_PU && c->selector == PU_HUE && v < 0) v += 360;
        ok = scalar(ctrl_fd(c), c->id, &v, true);
    }
    if (ok) s_manual_mask |= c->ipa_mask;
    _lock_release(&s_lock);
    ESP_LOGI(TAG, "SET entity=%u selector=%u value=%" PRIu32 " %s",
             c->entity, c->selector, value, ok ? "OK" : "FAILED");
    return ok ? CTRL_OK : CTRL_NOT_READY;
}

esp_err_t camera_controls_init(int camera_fd)
{
    s_camera = camera_fd;
    s_isp = open(ESP_VIDEO_ISP1_DEVICE_NAME, O_RDWR);
    if (s_isp < 0) return ESP_FAIL;
    esp_cam_sensor_format_t format = {0};
    if (ioctl(s_camera, VIDIOC_G_SENSOR_FMT, &format) || !format.isp_info) return ESP_FAIL;
    s_tline_ns = format.isp_info->isp_v1_info.tline_ns;
    if (!s_tline_ns) return ESP_FAIL;
    _lock_acquire(&s_lock);
    for (unsigned i = 0; i < sizeof(s_controls)/sizeof(s_controls[0]); ++i) {
        icg_control_t *c = &s_controls[i];
        if (c->id) {
            struct v4l2_query_ext_ctrl q = {.id = c->id};
            int32_t current = 0;
            if (ioctl(ctrl_fd(c), VIDIOC_QUERY_EXT_CTRL, &q) ||
                !scalar(ctrl_fd(c), c->id, &current, false)) continue;
            if (c->entity == ICG_CT && c->selector == CT_EXPOSURE) {
                s_exp_min = q.minimum; s_exp_max = q.maximum; s_exp_step = q.step ? q.step : 1;
                c->min = ((uint64_t)q.minimum*s_tline_ns + 99999)/100000;
                if (c->min < 1) c->min = 1;
                c->max = (uint64_t)q.maximum*s_tline_ns/100000;
                c->def = ((uint64_t)current*s_tline_ns + 50000)/100000;
            } else if (c->entity == ICG_CT && c->selector == CT_FOCUS) {
                /* Preserve the requested nominal 20–50 cm bracket. */
                if (q.minimum > c->min || q.maximum < c->max) continue;
            } else if (c->entity == ICG_PU && c->selector == PU_HUE) {
                c->min = -180; c->max = 180; c->def = current > 180 ? current-360 : current;
            } else {
                c->min = q.minimum; c->max = q.maximum;
                c->res = q.step ? q.step : 1; c->def = current;
            }
        } else if (c->entity == ICG_PU && c->selector == PU_SHARPNESS) {
            if (!blob(V4L2_CID_USER_ESP_ISP_SHARPEN, &s_sharp_base, sizeof(s_sharp_base), false)) continue;
        } else if (c->entity == ICG_PU && c->selector == PU_GAMMA) {
            if (!blob(V4L2_CID_USER_ESP_ISP_GAMMA_EXT, &s_gamma_base, sizeof(s_gamma_base), false)) continue;
        } else if (c->entity == ICG_PU && c->selector == PU_WB_COMPONENT) {
            esp_video_isp_wb_t balance;
            if (!blob(V4L2_CID_USER_ESP_ISP_WB, &balance, sizeof(balance), false)) continue;
            c->def = (uint32_t)lroundf(balance.blue_gain*1000) |
                     ((uint32_t)lroundf(balance.red_gain*1000) << 16);
        }
        c->enabled = true;
        ESP_LOGI(TAG, "control entity=%u selector=%u range=%" PRIi32 "..%" PRIi32,
                 c->entity, c->selector, c->min, c->max);
    }
    s_controls[0].enabled = camera_control_find(ICG_CT, CT_EXPOSURE) &&
                            camera_control_find(ICG_PU, PU_GAIN);
    s_controls[11].enabled = camera_control_find(ICG_PU, PU_WB_COMPONENT) != NULL;
    _lock_release(&s_lock);
    return ESP_OK;
}
