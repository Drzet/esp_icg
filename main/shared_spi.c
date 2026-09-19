#include "shared_spi.h"
#include "app_config.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* Excludes touch traffic while a card enters SPI mode. Ordinary SD transfers
 * and touch transfers are serialized by the SPI master driver itself. */
static SemaphoreHandle_t s_mount_lock;

esp_err_t shared_spi_init(void)
{
    s_mount_lock = xSemaphoreCreateMutex();
    if (!s_mount_lock) return ESP_ERR_NO_MEM;
    gpio_config_t cs = {
        .pin_bit_mask = (1ULL << ICG_TOUCH_PIN_CS) | (1ULL << ICG_SD_PIN_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    esp_err_t ret = gpio_config(&cs);
    if (ret != ESP_OK) return ret;
    gpio_set_level(ICG_TOUCH_PIN_CS, 1);
    gpio_set_level(ICG_SD_PIN_CS, 1);
    spi_bus_config_t bus = {
        .sclk_io_num = ICG_TOUCH_PIN_SCLK,
        .mosi_io_num = ICG_TOUCH_PIN_MOSI,
        .miso_io_num = ICG_TOUCH_PIN_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    return spi_bus_initialize(ICG_TOUCH_HOST, &bus, SPI_DMA_CH_AUTO);
}

bool shared_spi_touch_lock(void)
{
    return xSemaphoreTake(s_mount_lock, 0) == pdTRUE;
}
void shared_spi_mount_lock(void) { xSemaphoreTake(s_mount_lock, portMAX_DELAY); }
void shared_spi_unlock(void) { xSemaphoreGive(s_mount_lock); }
