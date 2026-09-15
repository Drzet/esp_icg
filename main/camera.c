#include "camera.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/ppa.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "linux/videodev2.h"
#include "wt_bsp.h"
#include "app_config.h"
#include "display.h"
#include "recorder.h"

static const char *TAG = "camera";
static wt_bsp_csi_t s_csi;
static ppa_client_handle_t s_ppa;
static uint16_t *s_preview;
static SemaphoreHandle_t s_preview_lock;
static volatile bool s_running;

static void camera_frame_cb(uint8_t *buf, uint32_t width, uint32_t height, size_t len, void *user_data)
{
    (void)user_data;
    if (!buf || !s_preview || !s_ppa || !s_running) return;

    recorder_submit_rgb565(buf, width, height, len);

    if (xSemaphoreTake(s_preview_lock, 0) != pdTRUE) return;

    uint32_t crop_w = width;
    uint32_t crop_h = height;
    if ((uint64_t)width * ICG_PREVIEW_HEIGHT > (uint64_t)height * ICG_LCD_WIDTH) {
        crop_w = (uint32_t)((uint64_t)height * ICG_LCD_WIDTH / ICG_PREVIEW_HEIGHT);
    } else {
        crop_h = (uint32_t)((uint64_t)width * ICG_PREVIEW_HEIGHT / ICG_LCD_WIDTH);
    }

    ppa_srm_oper_config_t op = {
        .in = {
            .buffer = buf,
            .pic_w = width,
            .pic_h = height,
            .block_w = crop_w,
            .block_h = crop_h,
            .block_offset_x = (width - crop_w) / 2,
            .block_offset_y = (height - crop_h) / 2,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer = s_preview,
            .buffer_size = ICG_LCD_WIDTH * ICG_PREVIEW_HEIGHT * sizeof(uint16_t),
            .pic_w = ICG_LCD_WIDTH,
            .pic_h = ICG_PREVIEW_HEIGHT,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x = (float)ICG_LCD_WIDTH / (float)crop_w,
        .scale_y = (float)ICG_PREVIEW_HEIGHT / (float)crop_h,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };

    esp_err_t ret = ppa_do_scale_rotate_mirror(s_ppa, &op);
    if (ret == ESP_OK) ret = display_draw_preview(s_preview);
    if (ret != ESP_OK) ESP_LOGW(TAG, "preview frame failed: %s", esp_err_to_name(ret));

    xSemaphoreGive(s_preview_lock);
}

esp_err_t camera_init(void)
{
    s_csi = wt_bsp_get_csi();
    if (!s_csi) {
        ESP_LOGE(TAG, "CSI unavailable; check OV5647 ribbon/orientation and camera configuration");
        return ESP_ERR_NOT_FOUND;
    }

    ppa_client_config_t cfg = {
        .oper_type = PPA_OPERATION_SRM,
        .max_pending_trans_num = 1,
    };
    ESP_RETURN_ON_ERROR(ppa_register_client(&cfg, &s_ppa), TAG, "PPA client");

    s_preview = heap_caps_malloc(ICG_LCD_WIDTH * ICG_PREVIEW_HEIGHT * sizeof(uint16_t),
                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_preview) return ESP_ERR_NO_MEM;

    s_preview_lock = xSemaphoreCreateMutex();
    if (!s_preview_lock) return ESP_ERR_NO_MEM;

    return wt_bsp_csi_set_pixel_format(s_csi, V4L2_PIX_FMT_RGB565);
}

esp_err_t camera_start(void)
{
    if (!s_csi) return ESP_ERR_INVALID_STATE;
    if (s_running) return ESP_OK;
    esp_err_t ret = wt_bsp_csi_start(s_csi, camera_frame_cb, NULL);
    if (ret == ESP_OK) s_running = true;
    return ret;
}

esp_err_t camera_stop(void)
{
    if (!s_csi) return ESP_ERR_INVALID_STATE;
    if (!s_running) return ESP_OK;
    s_running = false;
    recorder_set_active(false);
    return wt_bsp_csi_stop(s_csi);
}

bool camera_running(void)
{
    return s_running;
}
