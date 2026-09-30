#include "touch.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include "app_config.h"
#include "shared_spi.h"

#define XPT2046_CMD_X 0xD0
#define XPT2046_CMD_Y 0x90
#define TOUCH_SAMPLES 5

static const char *TAG = "touch";
static spi_device_handle_t s_touch;

static uint16_t median5(uint16_t v[TOUCH_SAMPLES])
{
    for (int i = 0; i < TOUCH_SAMPLES - 1; ++i) {
        for (int j = i + 1; j < TOUCH_SAMPLES; ++j) {
            if (v[j] < v[i]) {
                uint16_t t = v[i];
                v[i] = v[j];
                v[j] = t;
            }
        }
    }
    return v[TOUCH_SAMPLES / 2];
}

static bool read_axis(uint8_t cmd, uint16_t *out)
{
    uint16_t values[TOUCH_SAMPLES];

    for (int i = 0; i < TOUCH_SAMPLES; ++i) {
        uint8_t tx[3] = {cmd, 0, 0};
        uint8_t rx[3] = {0};
        spi_transaction_t t = {
            .length = 24,
            .tx_buffer = tx,
            .rx_buffer = rx,
        };
        if (spi_device_polling_transmit(s_touch, &t) != ESP_OK) {
            return false;
        }
        values[i] = (uint16_t)(((uint16_t)rx[1] << 8 | rx[2]) >> 3) & 0x0fff;
    }

    *out = median5(values);
    return true;
}

static int map_axis(int raw, int raw_min, int raw_max, int out_max, bool invert)
{
    if (raw < raw_min) raw = raw_min;
    if (raw > raw_max) raw = raw_max;

    int value = (raw - raw_min) * out_max / (raw_max - raw_min);
    if (invert) value = out_max - value;
    return value;
}

esp_err_t touch_init(void)
{
    gpio_config_t irq_cfg = {
        .pin_bit_mask = 1ULL << ICG_TOUCH_PIN_IRQ,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    esp_err_t ret = gpio_config(&irq_cfg);
    if (ret != ESP_OK) return ret;

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = ICG_TOUCH_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = ICG_TOUCH_PIN_CS,
        .queue_size = 1,
    };
    ret = spi_bus_add_device(ICG_TOUCH_HOST, &dev_cfg, &s_touch);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "XPT2046 ready on SPI3: CS=%d IRQ=%d SCLK=%d MOSI=%d MISO=%d",
                 ICG_TOUCH_PIN_CS, ICG_TOUCH_PIN_IRQ, ICG_TOUCH_PIN_SCLK,
                 ICG_TOUCH_PIN_MOSI, ICG_TOUCH_PIN_MISO);
    }
    return ret;
}

bool touch_read(touch_point_t *point)
{
    if (!point || !s_touch || gpio_get_level(ICG_TOUCH_PIN_IRQ) != 0) {
        return false;
    }

    uint16_t raw_x, raw_y;
    if (!shared_spi_touch_lock()) return false;
    bool ok = read_axis(XPT2046_CMD_X, &raw_x) && read_axis(XPT2046_CMD_Y, &raw_y);
    shared_spi_unlock();
    if (!ok) {
        return false;
    }

    point->raw_x = raw_x;
    point->raw_y = raw_y;

#if ICG_TOUCH_SWAP_XY
    point->x = map_axis(raw_y, ICG_TOUCH_RAW_Y_MIN, ICG_TOUCH_RAW_Y_MAX,
                        ICG_LCD_WIDTH - 1, ICG_TOUCH_INVERT_X);
    point->y = map_axis(raw_x, ICG_TOUCH_RAW_X_MIN, ICG_TOUCH_RAW_X_MAX,
                        ICG_LCD_HEIGHT - 1, ICG_TOUCH_INVERT_Y);
#else
    point->x = map_axis(raw_x, ICG_TOUCH_RAW_X_MIN, ICG_TOUCH_RAW_X_MAX,
                        ICG_LCD_WIDTH - 1, ICG_TOUCH_INVERT_X);
    point->y = map_axis(raw_y, ICG_TOUCH_RAW_Y_MIN, ICG_TOUCH_RAW_Y_MAX,
                        ICG_LCD_HEIGHT - 1, ICG_TOUCH_INVERT_Y);
#endif
    return true;
}

bool touch_is_pressed(void)
{
    return s_touch && gpio_get_level(ICG_TOUCH_PIN_IRQ) == 0;
}
