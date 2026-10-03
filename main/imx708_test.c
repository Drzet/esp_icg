#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "linux/videodev2.h"
#include "isp_diag.h"
#include "uvc_stream.h"
#include "camera_controls.h"

#if CONFIG_ESP_VIDEO_ISP_PIPELINE_CONTROL_CAMERA_MOTOR
#error "USB prototype uses fixed manual focus; disable IPA motor control"
#endif
#define CAMERA_POWER_GPIO 0
#define CAMERA_SCCB_I2C_PORT 0
#define CAMERA_SCCB_SCL 8
#define CAMERA_SCCB_SDA 7
#define CAMERA_SCCB_FREQ_HZ 100000
#define CAMERA_BUFFER_COUNT 3
/* Existing standard-lens mapping; nominal 30 cm, not IR calibrated. */
#define FOCUS_CODE_30CM 552
static const char *TAG = "imx708_usb";

static const esp_video_init_csi_config_t s_csi_config[] = {
    {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port = CAMERA_SCCB_I2C_PORT,
                .scl_pin = CAMERA_SCCB_SCL,
                .sda_pin = CAMERA_SCCB_SDA,
            },
            .freq = CAMERA_SCCB_FREQ_HZ,
        },
        .reset_pin = -1,
        .pwdn_pin = -1,
    },
};

static const esp_video_init_cam_motor_config_t s_motor_config[] = {
    {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port = CAMERA_SCCB_I2C_PORT,
                .scl_pin = CAMERA_SCCB_SCL,
                .sda_pin = CAMERA_SCCB_SDA,
            },
            .freq = CAMERA_SCCB_FREQ_HZ,
        },
        .reset_pin = -1,
        .pwdn_pin = -1,
        .signal_pin = -1,
    },
};

static const esp_video_init_config_t s_video_config = {
    .csi = s_csi_config,
    .cam_motor = s_motor_config,
};

static esp_err_t camera_power_on(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << CAMERA_POWER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t ret = gpio_config(&cfg);
    if (ret != ESP_OK) return ret;

    gpio_set_level(CAMERA_POWER_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

static bool write_focus_ctrl(int fd, int32_t value)
{
    struct v4l2_ext_control ctrl = {
        .id = V4L2_CID_FOCUS_ABSOLUTE,
        .value = value,
    };
    struct v4l2_ext_controls ctrls = {
        .ctrl_class = V4L2_CID_CAMERA_CLASS,
        .count = 1,
        .controls = &ctrl,
    };
    return ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls) == 0;
}

static esp_err_t run_camera(int fd)
{
    uint8_t *buffers[CAMERA_BUFFER_COUNT] = {0};
    size_t lengths[CAMERA_BUFFER_COUNT] = {0};
    uint32_t count = 0;
    bool streaming = false;
    esp_err_t result = ESP_FAIL;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_format fmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) != 0) return ESP_FAIL;
    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB565 ||
        fmt.fmt.pix.width != ICG_UVC_WIDTH || fmt.fmt.pix.height != ICG_UVC_HEIGHT ||
        (fmt.fmt.pix.bytesperline && fmt.fmt.pix.bytesperline != ICG_UVC_WIDTH * 2)) {
        ESP_LOGE(TAG, "unsupported capture geometry/stride");
        return ESP_ERR_NOT_SUPPORTED;
    }
    /* No VIDIOC_S_PARM: preserve the sensor's native delivery, with no 14 fps skip. */
    struct v4l2_streamparm parm = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
    if (ioctl(fd, VIDIOC_G_PARM, &parm) == 0 && parm.parm.capture.timeperframe.numerator) {
        ESP_LOGI(TAG, "native capture interval: %u/%u s",
                 parm.parm.capture.timeperframe.numerator,
                 parm.parm.capture.timeperframe.denominator);
    }
    struct v4l2_requestbuffers req = {
        .count = CAMERA_BUFFER_COUNT, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0 || req.count < 2 ||
        req.count > CAMERA_BUFFER_COUNT) return ESP_FAIL;
    count = req.count;
    for (uint32_t i = 0; i < count; ++i) {
        struct v4l2_buffer b = {
            .type = type, .memory = V4L2_MEMORY_MMAP, .index = i,
        };
        if (ioctl(fd, VIDIOC_QUERYBUF, &b) != 0) goto cleanup;
        lengths[i] = b.length;
        buffers[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        if (buffers[i] == MAP_FAILED) { buffers[i] = NULL; goto cleanup; }
    }
    if (!write_focus_ctrl(fd, FOCUS_CODE_30CM)) {
        ESP_LOGE(TAG, "initial focus failed: errno=%d", errno);
        goto cleanup;
    }
    result = camera_controls_init(fd);
    if (result != ESP_OK) goto cleanup;
    result = uvc_stream_init();
    if (result != ESP_OK) goto cleanup;
    ESP_LOGI(TAG, "USB ready: 1080p q80, nominal 30 cm focus, automatic exposure/gain");
    while (true) {
        bool wanted = uvc_stream_active();
        if (!wanted) {
            if (streaming) {
                if (ioctl(fd, VIDIOC_STREAMOFF, &type) != 0) goto capture_error;
                streaming = false;
                uvc_stream_discard_ready();
                ESP_LOGI(TAG, "camera stopped: USB idle/disconnected");
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (!streaming) {
            /* STREAMOFF clears the driver's queued/done lists. Requeue on each start. */
            for (uint32_t i = 0; i < count; ++i) {
                struct v4l2_buffer b = {
                    .type = type, .memory = V4L2_MEMORY_MMAP, .index = i,
                };
                if (ioctl(fd, VIDIOC_QBUF, &b) != 0) goto capture_error;
            }
            if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) goto capture_error;
            streaming = true;
            ESP_LOGI(TAG, "camera streaming: native delivery, no frame skipping configured");
        }
        struct v4l2_buffer b = { .type = type, .memory = V4L2_MEMORY_MMAP };
        if (ioctl(fd, VIDIOC_DQBUF, &b) != 0) goto capture_error;
        if (b.index >= count || b.bytesused > lengths[b.index]) goto capture_error;
        /* Reserve JPEG space before encoding. If USB is behind, skip raw frames.
         * The camera buffer stays dequeued until hardware JPEG has finished. */
        uvc_stream_submit(buffers[b.index], b.bytesused);
        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) goto capture_error;
    }
capture_error:
    ESP_LOGE(TAG, "capture failed: errno=%d", errno);
    result = ESP_FAIL;
    uvc_stream_disable();
cleanup:
    if (streaming) ioctl(fd, VIDIOC_STREAMOFF, &type);
    for (uint32_t i = 0; i < count; ++i) {
        if (buffers[i]) munmap(buffers[i], lengths[i]);
    }
    return result;
}

void app_main(void)
{
    ESP_LOGI(TAG, "IMX708 -> ISP/IPA -> hardware JPEG -> USB HS UVC");
    ESP_ERROR_CHECK(camera_power_on());
    ESP_ERROR_CHECK(esp_video_init_with_flags(&s_video_config,
                    ESP_VIDEO_INIT_FLAGS_MIPI_CSI | ESP_VIDEO_INIT_FLAGS_ISP |
                    ESP_VIDEO_INIT_FLAGS_MOTOR));
    esp_err_t diag_ret = isp_diag_start();
    if (diag_ret != ESP_OK) ESP_LOGW(TAG, "ISP diagnostics: %s", esp_err_to_name(diag_ret));
    int fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDONLY);
    if (fd < 0) { ESP_LOGE(TAG, "camera open failed: errno=%d", errno); return; }
    esp_err_t ret = run_camera(fd);
    ESP_LOGE(TAG, "camera task ended: %s", esp_err_to_name(ret));
    close(fd);
}
