#include <inttypes.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "camera_stream.h"
#include "display.h"
#include "recorder.h"
#include "shared_spi.h"
#include "touch.h"

#define PREVIEW_CROP_WIDTH 1080
#define PREVIEW_CROP_X ((P4D_CAMERA_WIDTH - PREVIEW_CROP_WIDTH) / 2)
#define PREVIEW_BUFFER_COUNT 3
#define PREVIEW_BUFFER_PIXELS (ICG_LCD_WIDTH * ICG_PREVIEW_HEIGHT)
#define BOARD_I2C_PORT I2C_NUM_0
#define BOARD_I2C_SDA 7
#define BOARD_I2C_SCL 8
#define DISPLAY_TASK_STACK 4096
#define RECORD_TASK_STACK 2048
#define TOUCH_TASK_STACK 3072
#define TOUCH_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define FAN_GPIO 37

/* Two horizontal 156-pixel control bands with an 8-pixel dead gap. */
#define TOUCH_ZONE_HEIGHT 156
#define TOUCH_ZONE_GAP 8

/* Preserve the old slider behaviour: rightmost 10% is a hard maximum plateau. */
#define CONTROL_MAX_PLATEAU_X ((ICG_LCD_WIDTH * 90) / 100)

static const char *TAG = "ov9281_app";

static uint16_t *s_preview[PREVIEW_BUFFER_COUNT];
static TaskHandle_t s_display_task;
static portMUX_TYPE s_preview_lock = portMUX_INITIALIZER_UNLOCKED;
static int s_latest_preview = -1;
static int s_display_preview = -1;
static volatile uint32_t s_capture_frames;
static volatile uint32_t s_display_frames;
static volatile uint32_t s_skipped_previews;
static bool s_recorder_ready;
static bool s_touch_available;
static p4d_camera_control_info_t s_control_info;

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

/* 1080x720 centre crop -> 480x320. Both axes scale exactly by 4/9. */
static void make_preview(const uint8_t *gray, uint16_t *out)
{
    for (uint32_t y = 0; y < ICG_PREVIEW_HEIGHT; ++y) {
        const uint32_t sy = (y * 9U) / 4U;
        const uint8_t *src =
            gray + (size_t)sy * P4D_CAMERA_WIDTH + PREVIEW_CROP_X;
        uint16_t *dst = out + (size_t)y * ICG_LCD_WIDTH;

        for (uint32_t x = 0; x < ICG_LCD_WIDTH; ++x) {
            const uint32_t sx = (x * 9U) / 4U;
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

static void camera_frame(const uint8_t *data,
                         size_t len,
                         uint32_t width,
                         uint32_t height,
                         void *user_ctx)
{
    (void)user_ctx;

    ++s_capture_frames;

    if (width != P4D_CAMERA_WIDTH || height != P4D_CAMERA_HEIGHT ||
        len < (size_t)P4D_CAMERA_WIDTH * P4D_CAMERA_HEIGHT) {
        static bool warned;
        if (!warned) {
            ESP_LOGE(TAG, "unexpected frame: %lux%lu len=%u",
                     (unsigned long)width, (unsigned long)height, (unsigned)len);
            warned = true;
        }
        return;
    }

    if (s_recorder_ready) {
        recorder_submit(data, len, P4D_CAMERA_WIDTH);
    }

    int preview_index = preview_acquire();
    if (preview_index >= 0) {
        make_preview(data, s_preview[preview_index]);
        preview_publish(preview_index);
    } else {
        ++s_skipped_previews;
    }

    if ((s_capture_frames % 50U) == 0) {
        ESP_LOGI(TAG,
                 "live: capture=%" PRIu32 " display=%" PRIu32
                 " skipped_preview=%" PRIu32,
                 s_capture_frames, s_display_frames, s_skipped_previews);
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
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << ICG_RECORD_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&cfg);
    if (ret != ESP_OK) return ret;

    return xTaskCreate(record_button_task, "rec_button", RECORD_TASK_STACK, NULL,
                       tskIDLE_PRIORITY + 1, NULL) == pdPASS
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}

static int touch_zone_from_y(int y)
{
    if (y >= 0 && y < TOUCH_ZONE_HEIGHT) return 0;

    const int lower = TOUCH_ZONE_HEIGHT + TOUCH_ZONE_GAP;
    if (y >= lower && y < lower + TOUCH_ZONE_HEIGHT) return 1;

    return -1;
}

static int32_t control_value_from_x(int x, int32_t minimum, int32_t maximum, int32_t step)
{
    if (x < 0) x = 0;
    if (x >= ICG_LCD_WIDTH) x = ICG_LCD_WIDTH - 1;
    if (step < 1) step = 1;
    if (maximum <= minimum) return minimum;

    if (x >= CONTROL_MAX_PLATEAU_X) return maximum;

    int32_t count = (maximum - minimum) / step + 1;
    if (count <= 1) return minimum;
    if (count == 2) return minimum;

    const int32_t non_max_count = count - 1;
    int32_t index =
        (int32_t)(((int64_t)x * (non_max_count - 1) +
                   (CONTROL_MAX_PLATEAU_X - 1) / 2) /
                  (CONTROL_MAX_PLATEAU_X - 1));
    int32_t value = minimum + index * step;
    return value > maximum ? maximum : value;
}

static void touch_task(void *arg)
{
    (void)arg;

    int active_zone = -1;
    int32_t active_value = INT32_MIN;
    unsigned release_samples = 0;

    while (true) {
        touch_point_t p;
        if (!touch_read(&p)) {
            if (!touch_is_pressed() && ++release_samples >= 3) {
                active_zone = -1;
                active_value = INT32_MIN;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        release_samples = 0;
        int zone = touch_zone_from_y(p.y);
        if (zone < 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        int32_t value;
        if (zone == 0) {
            value = control_value_from_x(
                p.x,
                s_control_info.exposure_min,
                s_control_info.exposure_max,
                s_control_info.exposure_step);
        } else {
            value = control_value_from_x(
                p.x,
                s_control_info.gain_min,
                s_control_info.gain_max,
                1);
        }

        if (zone == active_zone && value == active_value) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        active_zone = zone;
        active_value = value;

        if (zone == 0) {
            p4d_camera_request_exposure(value);
            ESP_LOGI(TAG, "touch EXP=%ld (%.1f ms) raw=%u,%u xy=%d,%d",
                     (long)value, (double)value / 10.0,
                     p.raw_x, p.raw_y, p.x, p.y);
        } else {
            p4d_camera_request_gain(value);
            ESP_LOGI(TAG, "touch GAIN idx=%ld raw=%u,%u xy=%d,%d",
                     (long)value, p.raw_x, p.raw_y, p.x, p.y);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "OV9281 1280x720 RAW8 -> SPI preview + grayscale JPEG/AVI");

    ESP_ERROR_CHECK(display_init());
    ESP_ERROR_CHECK(shared_spi_init());

    esp_err_t touch_ret = touch_init();
    s_touch_available = touch_ret == ESP_OK;
    if (!s_touch_available) {
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

    esp_err_t rec_ret = recorder_init(P4D_CAMERA_WIDTH, P4D_CAMERA_HEIGHT);
    if (rec_ret == ESP_OK) {
        s_recorder_ready = true;
        ESP_ERROR_CHECK(init_record_button());
        ESP_LOGI(TAG, "record/stop button: GPIO%d to GND", ICG_RECORD_BUTTON_GPIO);
    } else {
        ESP_LOGE(TAG, "recorder unavailable: %s", esp_err_to_name(rec_ret));
    }

    i2c_master_bus_handle_t i2c_bus = NULL;
    ESP_ERROR_CHECK(shared_i2c_init(&i2c_bus));
    ESP_ERROR_CHECK(p4d_camera_init(i2c_bus));
    ESP_ERROR_CHECK(p4d_camera_start(camera_frame, NULL));

    if (s_touch_available) {
        ESP_ERROR_CHECK(p4d_camera_get_control_info(&s_control_info));
        if (xTaskCreate(touch_task, "touch", TOUCH_TASK_STACK, NULL,
                        TOUCH_TASK_PRIORITY, NULL) != pdPASS) {
            ESP_LOGE(TAG, "touch task creation failed");
        } else {
            ESP_LOGI(TAG,
                     "touch controls: top=exposure bottom=gain, %d px dead gap",
                     TOUCH_ZONE_GAP);
        }
    }

    const gpio_config_t fan_cfg = {
        .pin_bit_mask = 1ULL << FAN_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&fan_cfg));
    ESP_ERROR_CHECK(gpio_set_level(FAN_GPIO, 1));
    ESP_LOGI(TAG, "fan on: GPIO%d high", FAN_GPIO);

    ESP_LOGI(TAG, "initialization complete");
}
