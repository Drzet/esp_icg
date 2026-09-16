#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/ppa.h"
#include "esp_cache.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
#include "esp_video_ioctl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imx708.h"
#include "linux/videodev2.h"

#include "app_config.h"
#include "display.h"

#define CAMERA_POWER_GPIO      0
#define CAMERA_SCCB_I2C_PORT   0
#define CAMERA_SCCB_SCL        8
#define CAMERA_SCCB_SDA        7
#define CAMERA_SCCB_FREQ_HZ    100000
#define CAMERA_BUFFER_COUNT    3

#define CAMERA_MODE_INDEX      0
#define CAMERA_WIDTH           1920
#define CAMERA_HEIGHT          1080

#define PREVIEW_CROP_WIDTH     1536
#define PREVIEW_CROP_HEIGHT    1024
#define PREVIEW_SCALE          (5.0f / 16.0f)
#define PPA_BUFFER_ALIGNMENT   128
#define PREVIEW_BUFFER_SIZE    (ICG_LCD_WIDTH * ICG_LCD_HEIGHT * sizeof(uint16_t))

static const char *TAG = "imx708_preview";
static ppa_client_handle_t s_ppa;
static uint16_t *s_preview;

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
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "camera power gpio");
    ESP_RETURN_ON_ERROR(gpio_set_level(CAMERA_POWER_GPIO, 1), TAG, "camera power on");
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

static esp_err_t select_1080p_rgb565(int fd, struct v4l2_format *fmt_out)
{
    const esp_cam_sensor_format_t *want = imx708_format_by_index(CAMERA_MODE_INDEX);
    if (!want) {
        ESP_LOGE(TAG, "IMX708 mode %d is unavailable", CAMERA_MODE_INDEX);
        return ESP_ERR_NOT_FOUND;
    }
    if (want->width != CAMERA_WIDTH || want->height != CAMERA_HEIGHT) {
        ESP_LOGE(TAG, "mode %d is %ux%u, expected %dx%d",
                 CAMERA_MODE_INDEX, want->width, want->height, CAMERA_WIDTH, CAMERA_HEIGHT);
        return ESP_ERR_INVALID_SIZE;
    }

    if (ioctl(fd, VIDIOC_S_SENSOR_FMT, (void *)want) != 0) {
        ESP_LOGE(TAG, "VIDIOC_S_SENSOR_FMT failed: errno=%d", errno);
        return ESP_FAIL;
    }

    struct v4l2_format fmt = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    };
    fmt.fmt.pix.width = CAMERA_WIDTH;
    fmt.fmt.pix.height = CAMERA_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "VIDIOC_S_FMT failed: errno=%d", errno);
        return ESP_FAIL;
    }

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "VIDIOC_G_FMT failed: errno=%d", errno);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "camera format: %" PRIu32 "x%" PRIu32 " fourcc=%c%c%c%c",
             fmt.fmt.pix.width, fmt.fmt.pix.height,
             (char)(fmt.fmt.pix.pixelformat & 0xff),
             (char)((fmt.fmt.pix.pixelformat >> 8) & 0xff),
             (char)((fmt.fmt.pix.pixelformat >> 16) & 0xff),
             (char)((fmt.fmt.pix.pixelformat >> 24) & 0xff));

    if (fmt.fmt.pix.width != CAMERA_WIDTH || fmt.fmt.pix.height != CAMERA_HEIGHT ||
        fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB565) {
        ESP_LOGE(TAG, "expected 1920x1080 RGB565 from ISP");
        return ESP_ERR_NOT_SUPPORTED;
    }

    *fmt_out = fmt;
    return ESP_OK;
}

static esp_err_t preview_frame(const uint8_t *frame, size_t len)
{
    const size_t expected = (size_t)CAMERA_WIDTH * CAMERA_HEIGHT * sizeof(uint16_t);
    if (!frame || len < expected) {
        return ESP_ERR_INVALID_SIZE;
    }

    const uint32_t crop_x = (CAMERA_WIDTH - PREVIEW_CROP_WIDTH) / 2;
    const uint32_t crop_y = (CAMERA_HEIGHT - PREVIEW_CROP_HEIGHT) / 2;

    ppa_srm_oper_config_t op = {
        .in = {
            .buffer = frame,
            .pic_w = CAMERA_WIDTH,
            .pic_h = CAMERA_HEIGHT,
            .block_w = PREVIEW_CROP_WIDTH,
            .block_h = PREVIEW_CROP_HEIGHT,
            .block_offset_x = crop_x,
            .block_offset_y = crop_y,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer = s_preview,
            .buffer_size = PREVIEW_BUFFER_SIZE,
            .pic_w = ICG_LCD_WIDTH,
            .pic_h = ICG_LCD_HEIGHT,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_180,
        .scale_x = PREVIEW_SCALE,
        .scale_y = PREVIEW_SCALE,
        .mirror_x = true,
        .mirror_y = false,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };

    /* PPA owns s_preview until this blocking call returns. */
    ESP_RETURN_ON_ERROR(ppa_do_scale_rotate_mirror(s_ppa, &op), TAG, "PPA");

    /* display_draw_preview() waits for every LCD SPI transfer to complete.
     * Only after it returns may the next PPA frame reuse s_preview. */
    return display_draw_preview(s_preview);
}

static esp_err_t run_preview(int fd)
{
    struct v4l2_format fmt;
    ESP_RETURN_ON_ERROR(select_1080p_rgb565(fd, &fmt), TAG, "camera format");

    uint8_t *buffers[CAMERA_BUFFER_COUNT] = {0};
    size_t lengths[CAMERA_BUFFER_COUNT] = {0};

    struct v4l2_requestbuffers req = {
        .count = CAMERA_BUFFER_COUNT,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0 || req.count < 2) {
        ESP_LOGE(TAG, "VIDIOC_REQBUFS failed: errno=%d count=%" PRIu32, errno, req.count);
        return ESP_FAIL;
    }

    uint32_t count = req.count > CAMERA_BUFFER_COUNT ? CAMERA_BUFFER_COUNT : req.count;
    for (uint32_t i = 0; i < count; ++i) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };
        if (ioctl(fd, VIDIOC_QUERYBUF, &b) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QUERYBUF[%" PRIu32 "] failed: errno=%d", i, errno);
            return ESP_FAIL;
        }

        lengths[i] = b.length;
        buffers[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        if (buffers[i] == MAP_FAILED) {
            buffers[i] = NULL;
            ESP_LOGE(TAG, "mmap[%" PRIu32 "] failed", i);
            return ESP_FAIL;
        }

        /* The CSI DMA will overwrite this PSRAM. Remove any dirty cache lines
         * left by the previous owner before handing the buffer to hardware. */
        size_t sync_len = b.length & ~(size_t)(PPA_BUFFER_ALIGNMENT - 1);
        if (sync_len) {
            ESP_RETURN_ON_ERROR(esp_cache_msync(buffers[i], sync_len,
                                                ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                                                ESP_CACHE_MSYNC_FLAG_INVALIDATE),
                                TAG, "camera buffer cache sync");
        }

        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF[%" PRIu32 "] failed: errno=%d", i, errno);
            return ESP_FAIL;
        }
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "VIDIOC_STREAMON failed: errno=%d", errno);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG,
             "live preview: 1920x1080 -> centered 1536x1024 -> 5/16 -> 480x320 RGB565 -> ILI9488");

    uint32_t frames = 0;
    esp_err_t result = ESP_OK;
    while (true) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(fd, VIDIOC_DQBUF, &b) != 0) {
            ESP_LOGE(TAG, "VIDIOC_DQBUF failed: errno=%d", errno);
            result = ESP_FAIL;
            break;
        }

        if (b.index >= count || !buffers[b.index]) {
            ESP_LOGE(TAG, "invalid camera buffer index %" PRIu32, b.index);
            result = ESP_FAIL;
            break;
        }

        esp_err_t ret = preview_frame(buffers[b.index], b.bytesused);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "preview failed: %s", esp_err_to_name(ret));
            result = ret;
            break;
        }

        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF failed: errno=%d", errno);
            result = ESP_FAIL;
            break;
        }

        ++frames;
        if ((frames % 100) == 0) {
            ESP_LOGI(TAG, "preview frames=%" PRIu32 " bytes=%" PRIu32, frames, b.bytesused);
        }
    }

    ioctl(fd, VIDIOC_STREAMOFF, &type);
    for (uint32_t i = 0; i < count; ++i) {
        if (buffers[i]) {
            munmap(buffers[i], lengths[i]);
        }
    }
    return result;
}

void app_main(void)
{
    ESP_LOGI(TAG, "IMX708 live preview: ISP/IPA/AF -> PPA -> ILI9488");

    ESP_ERROR_CHECK(display_init());
    ESP_ERROR_CHECK(camera_power_on());

    /* Keep the snapshot baseline's full video initialisation path, including
     * the DW9807 motor config and the ISP pipeline controller. */
    ESP_ERROR_CHECK(esp_video_init(&s_video_config));

    ppa_client_config_t ppa_cfg = {
        .oper_type = PPA_OPERATION_SRM,
        .max_pending_trans_num = 1,
    };
    ESP_ERROR_CHECK(ppa_register_client(&ppa_cfg, &s_ppa));

    s_preview = heap_caps_aligned_alloc(PPA_BUFFER_ALIGNMENT, PREVIEW_BUFFER_SIZE,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_preview) {
        ESP_LOGE(TAG, "aligned preview buffer allocation failed");
        return;
    }

    int fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "open %s failed: errno=%d", ESP_VIDEO_MIPI_CSI_DEVICE_NAME, errno);
        return;
    }

    esp_err_t ret = run_preview(fd);
    close(fd);
    ESP_LOGE(TAG, "preview stopped: %s", esp_err_to_name(ret));
}
