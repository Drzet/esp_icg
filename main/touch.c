#include "touch.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "app_config.h"

static uint16_t xpt_read12(uint8_t command)
{
    uint16_t value = 0;
    gpio_set_level(ICG_TOUCH_PIN_CS, 0);

    for (int i = 7; i >= 0; --i) {
        gpio_set_level(ICG_TOUCH_PIN_MOSI, (command >> i) & 1);
        gpio_set_level(ICG_TOUCH_PIN_SCLK, 1);
        esp_rom_delay_us(1);
        gpio_set_level(ICG_TOUCH_PIN_SCLK, 0);
        esp_rom_delay_us(1);
    }

    /* One null bit, then 12 ADC bits, then trailing bits. */
    gpio_set_level(ICG_TOUCH_PIN_SCLK, 1);
    esp_rom_delay_us(1);
    gpio_set_level(ICG_TOUCH_PIN_SCLK, 0);

    for (int i = 11; i >= 0; --i) {
        gpio_set_level(ICG_TOUCH_PIN_SCLK, 1);
        esp_rom_delay_us(1);
        value |= (gpio_get_level(ICG_TOUCH_PIN_MISO) & 1) << i;
        gpio_set_level(ICG_TOUCH_PIN_SCLK, 0);
        esp_rom_delay_us(1);
    }

    gpio_set_level(ICG_TOUCH_PIN_CS, 1);
    return value;
}

static uint16_t clamp_map(uint16_t raw, int minv, int maxv, int outmax)
{
    if (raw <= minv) return 0;
    if (raw >= maxv) return outmax - 1;
    return (uint16_t)(((uint32_t)(raw - minv) * (outmax - 1)) / (maxv - minv));
}

esp_err_t touch_init(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << ICG_TOUCH_PIN_CS) | (1ULL << ICG_TOUCH_PIN_SCLK) | (1ULL << ICG_TOUCH_PIN_MOSI),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&out));

    gpio_config_t in = {
        .pin_bit_mask = (1ULL << ICG_TOUCH_PIN_MISO) | (1ULL << ICG_TOUCH_PIN_IRQ),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&in));

    gpio_set_level(ICG_TOUCH_PIN_CS, 1);
    gpio_set_level(ICG_TOUCH_PIN_SCLK, 0);
    gpio_set_level(ICG_TOUCH_PIN_MOSI, 0);
    return ESP_OK;
}

bool touch_read(uint16_t *x, uint16_t *y)
{
    if (!x || !y || gpio_get_level(ICG_TOUCH_PIN_IRQ) != 0) return false;

    uint32_t sx = 0, sy = 0;
    for (int i = 0; i < 5; ++i) {
        sx += xpt_read12(0xD0); /* X */
        sy += xpt_read12(0x90); /* Y */
    }
    uint16_t rx = (uint16_t)(sx / 5);
    uint16_t ry = (uint16_t)(sy / 5);

#if ICG_TOUCH_SWAP_XY
    uint16_t tmp = rx; rx = ry; ry = tmp;
#endif

    uint16_t px = clamp_map(rx, ICG_TOUCH_X_MIN, ICG_TOUCH_X_MAX, ICG_LCD_WIDTH);
    uint16_t py = clamp_map(ry, ICG_TOUCH_Y_MIN, ICG_TOUCH_Y_MAX, ICG_LCD_HEIGHT);

#if ICG_TOUCH_INVERT_X
    px = ICG_LCD_WIDTH - 1 - px;
#endif
#if ICG_TOUCH_INVERT_Y
    py = ICG_LCD_HEIGHT - 1 - py;
#endif

    *x = px;
    *y = py;
    return true;
}
