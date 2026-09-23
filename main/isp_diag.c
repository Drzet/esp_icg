#include "isp_diag.h"

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_video_device.h"
#include "esp_video_isp_ioctl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "linux/videodev2.h"

#define ISP_DIAG_TASK_STACK      4096
#define ISP_DIAG_TASK_PRIORITY   (tskIDLE_PRIORITY + 1)
#define ISP_DIAG_PERIOD_MS       1000

static const char *TAG = "isp_diag";
static bool s_started;

enum {
    WARN_WB       = 1u << 0,
    WARN_BF       = 1u << 1,
    WARN_DM       = 1u << 2,
    WARN_SHARPEN  = 1u << 3,
    WARN_GAMMA    = 1u << 4,
    WARN_CCM      = 1u << 5,
    WARN_COLOR    = 1u << 6,
};

static uint32_t s_warned;

static bool read_blob_ctrl(int fd, uint32_t id, void *dst, size_t size)
{
    struct v4l2_ext_control ctrl = {
        .id = id,
        .size = size,
        .p_u8 = (uint8_t *)dst,
    };
    struct v4l2_ext_controls ctrls = {
        .ctrl_class = V4L2_CID_USER_CLASS,
        .count = 1,
        .controls = &ctrl,
    };

    memset(dst, 0, size);
    return ioctl(fd, VIDIOC_G_EXT_CTRLS, &ctrls) == 0;
}

static bool read_scalar_ctrl(int fd, uint32_t id, int32_t *value)
{
    struct v4l2_ext_control ctrl = {
        .id = id,
    };
    struct v4l2_ext_controls ctrls = {
        .ctrl_class = V4L2_CTRL_CLASS_USER,
        .count = 1,
        .controls = &ctrl,
    };

    if (ioctl(fd, VIDIOC_G_EXT_CTRLS, &ctrls) != 0) {
        return false;
    }
    *value = ctrl.value;
    return true;
}

static void warn_read_once(uint32_t bit, const char *name)
{
    if ((s_warned & bit) == 0) {
        s_warned |= bit;
        ESP_LOGW(TAG, "%s readback unavailable (errno=%d)", name, errno);
    }
}

static void log_snapshot(int fd)
{
    esp_video_isp_wb_t wb;
    if (read_blob_ctrl(fd, V4L2_CID_USER_ESP_ISP_WB, &wb, sizeof(wb))) {
        ESP_LOGI(TAG, "WB: en=%d R=%.4f B=%.4f",
                 wb.enable, (double)wb.red_gain, (double)wb.blue_gain);
    } else {
        warn_read_once(WARN_WB, "WB");
    }

    esp_video_isp_bf_t bf;
    if (read_blob_ctrl(fd, V4L2_CID_USER_ESP_ISP_BF, &bf, sizeof(bf))) {
        ESP_LOGI(TAG, "BF: en=%d level=%u matrix=%u,%u,%u/%u,%u,%u/%u,%u,%u",
                 bf.enable, bf.level,
                 bf.matrix[0][0], bf.matrix[0][1], bf.matrix[0][2],
                 bf.matrix[1][0], bf.matrix[1][1], bf.matrix[1][2],
                 bf.matrix[2][0], bf.matrix[2][1], bf.matrix[2][2]);
    } else {
        warn_read_once(WARN_BF, "BF");
    }

    esp_video_isp_demosaic_t dm;
    if (read_blob_ctrl(fd, V4L2_CID_USER_ESP_ISP_DEMOSAIC, &dm, sizeof(dm))) {
        ESP_LOGI(TAG, "DEMOSAIC: en=%d gradient=%.4f",
                 dm.enable, (double)dm.gradient_ratio);
    } else {
        warn_read_once(WARN_DM, "DEMOSAIC");
    }

    esp_video_isp_sharpen_t sharpen;
    if (read_blob_ctrl(fd, V4L2_CID_USER_ESP_ISP_SHARPEN, &sharpen, sizeof(sharpen))) {
        ESP_LOGI(TAG,
                 "SHARPEN: en=%d ht=%u lt=%u hc=%.4f mc=%.4f matrix=%u,%u,%u/%u,%u,%u/%u,%u,%u",
                 sharpen.enable, sharpen.h_thresh, sharpen.l_thresh,
                 (double)sharpen.h_coeff, (double)sharpen.m_coeff,
                 sharpen.matrix[0][0], sharpen.matrix[0][1], sharpen.matrix[0][2],
                 sharpen.matrix[1][0], sharpen.matrix[1][1], sharpen.matrix[1][2],
                 sharpen.matrix[2][0], sharpen.matrix[2][1], sharpen.matrix[2][2]);
    } else {
        warn_read_once(WARN_SHARPEN, "SHARPEN");
    }

    esp_video_isp_ccm_t ccm;
    if (read_blob_ctrl(fd, V4L2_CID_USER_ESP_ISP_CCM, &ccm, sizeof(ccm))) {
        ESP_LOGI(TAG,
                 "CCM: en=%d [%.3f %.3f %.3f; %.3f %.3f %.3f; %.3f %.3f %.3f]",
                 ccm.enable,
                 (double)ccm.matrix[0][0], (double)ccm.matrix[0][1], (double)ccm.matrix[0][2],
                 (double)ccm.matrix[1][0], (double)ccm.matrix[1][1], (double)ccm.matrix[1][2],
                 (double)ccm.matrix[2][0], (double)ccm.matrix[2][1], (double)ccm.matrix[2][2]);
    } else {
        warn_read_once(WARN_CCM, "CCM");
    }

    esp_video_isp_gamma_ext_t gamma;
    if (read_blob_ctrl(fd, V4L2_CID_USER_ESP_ISP_GAMMA_EXT, &gamma, sizeof(gamma))) {
        const int mid = ISP_GAMMA_CURVE_POINTS_NUM / 2;
        const int last = ISP_GAMMA_CURVE_POINTS_NUM - 1;
        ESP_LOGI(TAG,
                 "GAMMA: en=%d flags=0x%lx "
                 "R=%u:%u,%u:%u,%u:%u G=%u:%u,%u:%u,%u:%u B=%u:%u,%u:%u,%u:%u",
                 gamma.enable, (unsigned long)gamma.flags,
                 gamma.red_points[0].x, gamma.red_points[0].y,
                 gamma.red_points[mid].x, gamma.red_points[mid].y,
                 gamma.red_points[last].x, gamma.red_points[last].y,
                 gamma.green_points[0].x, gamma.green_points[0].y,
                 gamma.green_points[mid].x, gamma.green_points[mid].y,
                 gamma.green_points[last].x, gamma.green_points[last].y,
                 gamma.blue_points[0].x, gamma.blue_points[0].y,
                 gamma.blue_points[mid].x, gamma.blue_points[mid].y,
                 gamma.blue_points[last].x, gamma.blue_points[last].y);
    } else {
        warn_read_once(WARN_GAMMA, "GAMMA");
    }

    int32_t brightness;
    int32_t contrast;
    int32_t saturation;
    int32_t hue;
    bool br_ok = read_scalar_ctrl(fd, V4L2_CID_BRIGHTNESS, &brightness);
    bool cn_ok = read_scalar_ctrl(fd, V4L2_CID_CONTRAST, &contrast);
    bool st_ok = read_scalar_ctrl(fd, V4L2_CID_SATURATION, &saturation);
    bool hue_ok = read_scalar_ctrl(fd, V4L2_CID_HUE, &hue);

    if (br_ok && cn_ok && st_ok && hue_ok) {
        ESP_LOGI(TAG, "COLOR: brightness=%ld contrast=%ld saturation=%ld hue=%ld",
                 (long)brightness, (long)contrast, (long)saturation, (long)hue);
    } else {
        warn_read_once(WARN_COLOR, "COLOR");
    }
}

static void isp_diag_task(void *arg)
{
    (void)arg;

    /*
     * esp_video_init_with_flags() has already initialized the ISP pipeline
     * before this task is created. Delay once so the camera stream can start
     * before the first snapshot.
     */
    vTaskDelay(pdMS_TO_TICKS(1000));

    int fd = open(ESP_VIDEO_ISP1_DEVICE_NAME, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "open %s failed: errno=%d", ESP_VIDEO_ISP1_DEVICE_NAME, errno);
        s_started = false;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "read-only ISP diagnostics started (%d ms)", ISP_DIAG_PERIOD_MS);

    while (true) {
        log_snapshot(fd);
        vTaskDelay(pdMS_TO_TICKS(ISP_DIAG_PERIOD_MS));
    }
}

esp_err_t isp_diag_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    if (xTaskCreate(isp_diag_task, "isp_diag", ISP_DIAG_TASK_STACK, NULL,
                    ISP_DIAG_TASK_PRIORITY, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    return ESP_OK;
}
