#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/ppa.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "linux/videodev2.h"

#include "app_config.h"
#include "display.h"

#define CAMERA_POWER_GPIO      0
#define CAMERA_SCCB_I2C_PORT   0
#define CAMERA_SCCB_SCL        8
#define CAMERA_SCCB_SDA        7
#define CAMERA_SCCB_FREQ_HZ    100000
#define CAMERA_BUFFER_COUNT    3
#define PPA_CACHE_LINE_SIZE    128
#define PREVIEW_BUFFER_SIZE    (ICG_LCD_WIDTH * ICG_PREVIEW_HEIGHT * sizeof(uint16_t))

/*
 * ESP32-P4 PPA scale factors are quantized to 1/16 steps. 1920x1080 ->
 * 480x280 cannot therefore use the naive ~0.259 scale: it is truncated to
 * 0.25, producing a smaller block and leaving stale strips at the right and
 * bottom. A centered 1536x896 crop scales exactly by 5/16 to 480x280.
 */
#define PREVIEW_CROP_WIDTH     1536
#define PREVIEW_CROP_HEIGHT    896
#define PREVIEW_SCALE          (5.0f / 16.0f)

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

static const esp_video_init_config_t s_video_config = {
    .csi = s_csi_config,
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

static esp_err_t preview_frame(const uint8_t *frame, uint32_t width, uint32_t height, size_t len)
{
    if (!frame || len < (size_t)width * height * sizeof(uint16_t)) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (width < PREVIEW_CROP_WIDTH || height < PREVIEW_CROP_HEIGHT) {
        return ESP_ERR_INVALID_SIZE;
    }

    const uint32_t crop_x = (width - PREVIEW_CROP_WIDTH) / 2;
    const uint32_t crop_y = (height - PREVIEW_CROP_HEIGHT) / 2;

    ppa_srm_oper_config_t op = {
        .in = {
            .buffer = (void *)frame,
            .pic_w = width,
            .pic_h = height,
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
            .pic_h = ICG_PREVIEW_HEIGHT,
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

    esp_err_t ret = ppa_do_scale_rotate_mirror(s_ppa, &op);
    if (ret != ESP_OK) return ret;

    return display_draw_preview(s_preview);
}

static esp_err_t run_preview(int fd)
{
    uint8_t *buffers[CAMERA_BUFFER_COUNT] = {0};
    size_t lengths[CAMERA_BUFFER_COUNT] = {0};

    struct v4l2_format fmt = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    };
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "VIDIOC_G_FMT failed: errno=%d", errno);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "format: %" PRIu32 "x%" PRIu32 " fourcc=%c%c%c%c",
             fmt.fmt.pix.width, fmt.fmt.pix.height,
             (char)(fmt.fmt.pix.pixelformat & 0xff),
             (char)((fmt.fmt.pix.pixelformat >> 8) & 0xff),
             (char)((fmt.fmt.pix.pixelformat >> 16) & 0xff),
             (char)((fmt.fmt.pix.pixelformat >> 24) & 0xff));

    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB565) {
        ESP_LOGE(TAG, "expected RGB565 from ISP, got 0x%08" PRIx32, fmt.fmt.pix.pixelformat);
        return ESP_ERR_NOT_SUPPORTED;
    }

    struct v4l2_requestbuffers req = {
        .count = CAMERA_BUFFER_COUNT,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0 || req.count < 2) {
        ESP_LOGE(TAG, "VIDIOC_REQBUFS failed: errno=%d", errno);
        return ESP_FAIL;
    }

    uint32_t count = req.count > CAMERA_BUFFER_COUNT ? CAMERA_BUFFER_COUNT : req.count;
    for (uint32_t i = 0; i < count; ++i) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };
        if (ioctl(fd, VIDIOC_QUERYBUF, &b) != 0) return ESP_FAIL;

        lengths[i] = b.length;
        buffers[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        if (buffers[i] == MAP_FAILED) {
            buffers[i] = NULL;
            return ESP_FAIL;
        }
        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) return ESP_FAIL;
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "VIDIOC_STREAMON failed: errno=%d", errno);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "live preview started: crop=%dx%d scale=5/16 rotation=180 mirror_x=1",
             PREVIEW_CROP_WIDTH, PREVIEW_CROP_HEIGHT);
    uint32_t frames = 0;

    while (true) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(fd, VIDIOC_DQBUF, &b) != 0) {
            ESP_LOGE(TAG, "VIDIOC_DQBUF failed: errno=%d", errno);
            break;
        }

        if (b.index >= count || !buffers[b.index]) {
            ESP_LOGE(TAG, "invalid camera buffer index");
            break;
        }

        esp_err_t ret = preview_frame(buffers[b.index], fmt.fmt.pix.width,
                                      fmt.fmt.pix.height, b.bytesused);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "preview failed: %s", esp_err_to_name(ret));
            break;
        }

        ++frames;
        if ((frames % 100) == 0) {
            ESP_LOGI(TAG, "preview frames=%" PRIu32 " bytes=%" PRIu32, frames, b.bytesused);
        }

        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF failed: errno=%d", errno);
            break;
        }
    }

    ioctl(fd, VIDIOC_STREAMOFF, &type);
    for (uint32_t i = 0; i < count; ++i) {
        if (buffers[i]) munmap(buffers[i], lengths[i]);
    }
    return ESP_FAIL;
}

void app_main(void)
{
    ESP_LOGI(TAG, "IMX708 -> ISP RGB565 -> PPA -> ILI9488 preview");

    ESP_ERROR_CHECK(display_init());
    ESP_ERROR_CHECK(camera_power_on());

    ESP_ERROR_CHECK(esp_video_init_with_flags(&s_video_config,
                                              ESP_VIDEO_INIT_FLAGS_MIPI_CSI |
                                              ESP_VIDEO_INIT_FLAGS_ISP));

    ppa_client_config_t ppa_cfg = {
        .oper_type = PPA_OPERATION_SRM,
        .max_pending_trans_num = 1,
    };
    ESP_ERROR_CHECK(ppa_register_client(&ppa_cfg, &s_ppa));

    s_preview = heap_caps_aligned_alloc(PPA_CACHE_LINE_SIZE, PREVIEW_BUFFER_SIZE,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_preview) {
        ESP_LOGE(TAG, "preview buffer allocation failed");
        return;
    }

    int fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "open %s failed: errno=%d", ESP_VIDEO_MIPI_CSI_DEVICE_NAME, errno);
        return;
    }

    run_preview(fd);
    close(fd);
}
