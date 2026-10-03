#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
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
#include "recorder.h"
#include "shared_spi.h"
#include "touch.h"

#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 400
#define CAMERA_BUFFER_COUNT 3
#define PREVIEW_CROP_WIDTH 600
#define PREVIEW_CROP_X ((CAMERA_WIDTH - PREVIEW_CROP_WIDTH) / 2)
#define PREVIEW_BUFFER_COUNT 3
#define PREVIEW_BUFFER_PIXELS (ICG_LCD_WIDTH * ICG_PREVIEW_HEIGHT)
#define BOARD_I2C_PORT I2C_NUM_0
#define BOARD_I2C_SDA 7
#define BOARD_I2C_SCL 8
#define DISPLAY_TASK_STACK 4096
#define RECORD_TASK_STACK 2048

static const char *TAG = "ov9281_app";

static uint16_t *s_preview[PREVIEW_BUFFER_COUNT];
static TaskHandle_t s_display_task;
static portMUX_TYPE s_preview_lock = portMUX_INITIALIZER_UNLOCKED;
static int s_latest_preview = -1;
static int s_display_preview = -1;
static volatile uint32_t s_capture_frames;
static volatile uint32_t s_display_frames;
static volatile uint32_t s_skipped_previews;

static esp_err_t shared_i2c_init(i2c_master_bus_handle_t *ret_bus)
{
    const i2c_master_bus_config_t cfg = {
        .i2c_port = BOARD_I2C_PORT,
        .sda_io_num = BOARD_I2C_SDA,
        .scl_io_num = BOARD_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&cfg, ret_bus);
}

static esp_err_t camera_init(i2c_master_bus_handle_t i2c_bus)
{
    esp_video_init_csi_config_t csi = {0};
    csi.sccb_config.init_sccb = false;
    csi.sccb_config.i2c_handle = i2c_bus;
    csi.sccb_config.freq = 400000;
    csi.reset_pin = -1;
    csi.pwdn_pin = -1;

    esp_video_init_csi_config_t csi_arr[] = { csi };
    esp_video_init_config_t video = {
        .csi = csi_arr,
    };

    vTaskDelay(pdMS_TO_TICKS(50));
    esp_err_t ret = esp_video_init(&video);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(200));
    return ESP_OK;
}

static inline uint16_t gray_to_rgb565(uint8_t g)
{
    return (uint16_t)(((uint16_t)(g >> 3) << 11) |
                      ((uint16_t)(g >> 2) << 5) |
                      (uint16_t)(g >> 3));
}

static int preview_acquire(void)
{
    int index = -1;
    portENTER_CRITICAL(&s_preview_lock);
    if (s_latest_preview < 0) {
        for (int i = 0; i < PREVIEW_BUFFER_COUNT; ++i) {
            if (i != s_display_preview) {
                index = i;
                break;
            }
        }
    }
    portEXIT_CRITICAL(&s_preview_lock);
    return index;
}

static void preview_publish(int index)
{
    portENTER_CRITICAL(&s_preview_lock);
    s_latest_preview = index;
    portEXIT_CRITICAL(&s_preview_lock);
    xTaskNotifyGive(s_display_task);
}

/* 600x400 centre crop -> 480x320. Both axes are exactly 4/5. */
static void make_preview(const uint8_t *gray, uint16_t *out)
{
    for (uint32_t y = 0; y < ICG_PREVIEW_HEIGHT; ++y) {
        const uint32_t sy = (y * 5U) / 4U;
        const uint8_t *src = gray + (size_t)sy * CAMERA_WIDTH + PREVIEW_CROP_X;
        uint16_t *dst = out + (size_t)y * ICG_LCD_WIDTH;
        for (uint32_t x = 0; x < ICG_LCD_WIDTH; ++x) {
            const uint32_t sx = (x * 5U) / 4U;
            dst[x] = gray_to_rgb565(src[sx]);
        }
    }
}

static void display_task(void *arg)
{
    (void)arg;
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int index = -1;
        portENTER_CRITICAL(&s_preview_lock);
        if (s_latest_preview >= 0) {
            index = s_latest_preview;
            s_latest_preview = -1;
            s_display_preview = index;
        }
        portEXIT_CRITICAL(&s_preview_lock);

        if (index < 0) continue;

        if (display_draw_preview(s_preview[index]) == ESP_OK) {
            ++s_display_frames;
        }

        portENTER_CRITICAL(&s_preview_lock);
        s_display_preview = -1;
        portEXIT_CRITICAL(&s_preview_lock);
    }
}

static void record_button_task(void *arg)
{
    (void)arg;
    int candidate = gpio_get_level(ICG_RECORD_BUTTON_GPIO);
    int stable = candidate;
    bool armed = false;
    TickType_t changed = xTaskGetTickCount();

    while (true) {
        int level = gpio_get_level(ICG_RECORD_BUTTON_GPIO);
        TickType_t now = xTaskGetTickCount();

        if (level != candidate) {
            candidate = level;
            changed = now;
        }
        if ((TickType_t)(now - changed) >= pdMS_TO_TICKS(40)) {
            if (candidate != stable) {
                stable = candidate;
                if (stable == 0 && armed) {
                    armed = false;
                    recorder_toggle();
                }
            }
            if (stable == 1) armed = true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static esp_err_t init_record_button(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << ICG_RECORD_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&cfg);
    if (ret != ESP_OK) return ret;
    return xTaskCreate(record_button_task, "rec_button", RECORD_TASK_STACK, NULL,
                       tskIDLE_PRIORITY + 1, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t run_camera(void)
{
    uint8_t *buffers[CAMERA_BUFFER_COUNT] = {0};
    size_t lengths[CAMERA_BUFFER_COUNT] = {0};

    int fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "failed to open %s errno=%d", ESP_VIDEO_MIPI_CSI_DEVICE_NAME, errno);
        return ESP_FAIL;
    }

    struct v4l2_format fmt = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .fmt.pix.width = CAMERA_WIDTH,
        .fmt.pix.height = CAMERA_HEIGHT,
        .fmt.pix.pixelformat = V4L2_PIX_FMT_SBGGR8,
    };
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "VIDIOC_S_FMT RAW8 640x400 failed errno=%d", errno);
        close(fd);
        return ESP_FAIL;
    }

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "VIDIOC_G_FMT failed errno=%d", errno);
        close(fd);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "camera: %" PRIu32 "x%" PRIu32 " fourcc=%c%c%c%c stride=%" PRIu32,
             fmt.fmt.pix.width, fmt.fmt.pix.height,
             (char)(fmt.fmt.pix.pixelformat & 0xff),
             (char)((fmt.fmt.pix.pixelformat >> 8) & 0xff),
             (char)((fmt.fmt.pix.pixelformat >> 16) & 0xff),
             (char)((fmt.fmt.pix.pixelformat >> 24) & 0xff),
             fmt.fmt.pix.bytesperline);

    if (fmt.fmt.pix.width != CAMERA_WIDTH || fmt.fmt.pix.height != CAMERA_HEIGHT ||
        fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_SBGGR8) {
        ESP_LOGE(TAG, "unexpected OV9281 format");
        close(fd);
        return ESP_ERR_NOT_SUPPORTED;
    }

    const size_t gray_stride = fmt.fmt.pix.bytesperline ?
                               fmt.fmt.pix.bytesperline : CAMERA_WIDTH;
    if (gray_stride != CAMERA_WIDTH) {
        ESP_LOGE(TAG, "RAW8 stride=%u is not tightly packed; expected 640",
                 (unsigned)gray_stride);
        close(fd);
        return ESP_ERR_NOT_SUPPORTED;
    }

    struct v4l2_requestbuffers req = {
        .count = CAMERA_BUFFER_COUNT,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0 || req.count < 2) {
        ESP_LOGE(TAG, "VIDIOC_REQBUFS failed errno=%d", errno);
        close(fd);
        return ESP_FAIL;
    }

    const uint32_t count = req.count > CAMERA_BUFFER_COUNT ? CAMERA_BUFFER_COUNT : req.count;
    for (uint32_t i = 0; i < count; ++i) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };
        if (ioctl(fd, VIDIOC_QUERYBUF, &b) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QUERYBUF %u failed errno=%d", (unsigned)i, errno);
            close(fd);
            return ESP_FAIL;
        }
        lengths[i] = b.length;
        buffers[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        if (buffers[i] == MAP_FAILED) {
            buffers[i] = NULL;
            ESP_LOGE(TAG, "mmap %u failed errno=%d", (unsigned)i, errno);
            close(fd);
            return ESP_FAIL;
        }
        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF %u failed errno=%d", (unsigned)i, errno);
            close(fd);
            return ESP_FAIL;
        }
    }

    esp_err_t rec_ret = recorder_init(CAMERA_WIDTH, CAMERA_HEIGHT);
    if (rec_ret != ESP_OK) {
        ESP_LOGE(TAG, "recorder unavailable: %s", esp_err_to_name(rec_ret));
    } else {
        ESP_ERROR_CHECK(init_record_button());
        ESP_LOGI(TAG, "record/stop button: GPIO%d to GND", ICG_RECORD_BUTTON_GPIO);
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "VIDIOC_STREAMON failed errno=%d", errno);
        close(fd);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OV9281 live: RAW8 640x400; preview crop 600x400 -> 480x320");

    while (true) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(fd, VIDIOC_DQBUF, &b) != 0) {
            if (errno == EAGAIN) {
                vTaskDelay(1);
                continue;
            }
            ESP_LOGE(TAG, "VIDIOC_DQBUF failed errno=%d", errno);
            break;
        }

        if (b.index >= count || !buffers[b.index]) {
            ESP_LOGE(TAG, "invalid camera buffer index=%u", (unsigned)b.index);
            break;
        }

        ++s_capture_frames;
        const uint8_t *gray = buffers[b.index];

        if (rec_ret == ESP_OK) {
            recorder_submit(gray, b.bytesused, gray_stride);
        }

        int preview_index = preview_acquire();
        if (preview_index >= 0) {
            make_preview(gray, s_preview[preview_index]);
            preview_publish(preview_index);
        } else {
            ++s_skipped_previews;
        }

        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF failed errno=%d", errno);
            break;
        }

        if ((s_capture_frames % 100U) == 0) {
            ESP_LOGI(TAG, "live: capture=%" PRIu32 " display=%" PRIu32
                     " skipped_preview=%" PRIu32,
                     s_capture_frames, s_display_frames, s_skipped_previews);
        }
    }

    ioctl(fd, VIDIOC_STREAMOFF, &type);
    for (uint32_t i = 0; i < count; ++i) {
        if (buffers[i]) munmap(buffers[i], lengths[i]);
    }
    close(fd);
    return ESP_FAIL;
}

void app_main(void)
{
    ESP_LOGI(TAG, "OV9281 -> RAW8 -> grayscale JPEG/AVI + ILI9488 preview");

    ESP_ERROR_CHECK(display_init());
    ESP_ERROR_CHECK(shared_spi_init());

    esp_err_t touch_ret = touch_init();
    if (touch_ret != ESP_OK) {
        ESP_LOGW(TAG, "touch unavailable: %s", esp_err_to_name(touch_ret));
    }

    for (int i = 0; i < PREVIEW_BUFFER_COUNT; ++i) {
        s_preview[i] = heap_caps_malloc(PREVIEW_BUFFER_PIXELS * sizeof(uint16_t),
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_preview[i]) {
            ESP_LOGE(TAG, "preview buffer allocation failed");
            return;
        }
    }

    if (xTaskCreate(display_task, "display", DISPLAY_TASK_STACK, NULL,
                    tskIDLE_PRIORITY + 1, &s_display_task) != pdPASS) {
        ESP_LOGE(TAG, "display task creation failed");
        return;
    }

    i2c_master_bus_handle_t i2c_bus = NULL;
    ESP_ERROR_CHECK(shared_i2c_init(&i2c_bus));
    ESP_ERROR_CHECK(camera_init(i2c_bus));

    esp_err_t ret = run_camera();
    ESP_LOGE(TAG, "camera stopped: %s", esp_err_to_name(ret));
}
