#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "linux/videodev2.h"

#define CAMERA_POWER_GPIO      0
#define CAMERA_SCCB_I2C_PORT   0
#define CAMERA_SCCB_SCL        8
#define CAMERA_SCCB_SDA        7
#define CAMERA_SCCB_FREQ_HZ    100000
#define CAMERA_BUFFER_COUNT    3
#define CAMERA_TEST_FRAMES     60

static const char *TAG = "imx708_test";

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

static void log_fourcc(uint32_t f)
{
    ESP_LOGI(TAG, "pixel format: %c%c%c%c (0x%08" PRIx32 ")",
             (char)(f & 0xff), (char)((f >> 8) & 0xff),
             (char)((f >> 16) & 0xff), (char)((f >> 24) & 0xff), f);
}

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

static esp_err_t capture_test(int fd)
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

    ESP_LOGI(TAG,
             "format: %" PRIu32 "x%" PRIu32 ", bytesperline=%" PRIu32 ", sizeimage=%" PRIu32,
             fmt.fmt.pix.width, fmt.fmt.pix.height,
             fmt.fmt.pix.bytesperline, fmt.fmt.pix.sizeimage);
    log_fourcc(fmt.fmt.pix.pixelformat);

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
            ESP_LOGE(TAG, "mmap[%" PRIu32 "] failed: errno=%d", i, errno);
            return ESP_FAIL;
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

    ESP_LOGI(TAG, "streaming started; capturing %d frames", CAMERA_TEST_FRAMES);

    esp_err_t result = ESP_OK;
    for (int frame = 0; frame < CAMERA_TEST_FRAMES; ++frame) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };

        if (ioctl(fd, VIDIOC_DQBUF, &b) != 0) {
            ESP_LOGE(TAG, "VIDIOC_DQBUF failed at frame %d: errno=%d", frame, errno);
            result = ESP_FAIL;
            break;
        }

        if (b.index >= count || !buffers[b.index]) {
            ESP_LOGE(TAG, "invalid buffer index %" PRIu32, b.index);
            result = ESP_FAIL;
            break;
        }

        const uint8_t *p = buffers[b.index];
        uint32_t step = b.bytesused > 4096 ? b.bytesused / 4096 : 1;
        uint64_t sum = 0;
        uint32_t samples = 0;
        for (uint32_t i = 0; i < b.bytesused; i += step) {
            sum += p[i];
            ++samples;
        }

        if ((frame % 10) == 0 || frame == CAMERA_TEST_FRAMES - 1) {
            ESP_LOGI(TAG,
                     "frame %d: seq=%" PRIu32 " bytes=%" PRIu32 " avg=%" PRIu32,
                     frame, b.sequence, b.bytesused,
                     samples ? (uint32_t)(sum / samples) : 0);
        }

        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF recycle failed: errno=%d", errno);
            result = ESP_FAIL;
            break;
        }
    }

    ioctl(fd, VIDIOC_STREAMOFF, &type);
    for (uint32_t i = 0; i < count; ++i) {
        if (buffers[i]) munmap(buffers[i], lengths[i]);
    }
    return result;
}

void app_main(void)
{
    ESP_LOGI(TAG, "IMX708 NoIR fixed-focus bring-up");

    esp_err_t ret = camera_power_on();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "camera power failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_video_init_with_flags(&s_video_config,
                                    ESP_VIDEO_INIT_FLAGS_MIPI_CSI |
                                    ESP_VIDEO_INIT_FLAGS_ISP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_video init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "esp_video init OK");

    int fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "open %s failed: errno=%d", ESP_VIDEO_MIPI_CSI_DEVICE_NAME, errno);
        esp_video_deinit();
        return;
    }

    struct v4l2_capability cap = {0};
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
        ESP_LOGI(TAG, "driver=%s card=%s", cap.driver, cap.card);
    }

    ret = capture_test(fd);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "==== IMX708 BRING-UP PASSED ====");
    } else {
        ESP_LOGE(TAG, "==== IMX708 BRING-UP FAILED ====");
    }

    close(fd);
    esp_video_deinit();
}
