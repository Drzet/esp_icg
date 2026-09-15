#include "storage.h"

#include <stdio.h>
#include "sdkconfig.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "app_config.h"

static const char *TAG = "storage";
static const char *MOUNT = "/sdcard";
static bool s_ready;
static sdmmc_card_t *s_card;

esp_err_t storage_init(void)
{
    if (CONFIG_ICG_SD_SCLK_GPIO < 0 || CONFIG_ICG_SD_MOSI_GPIO < 0 ||
        CONFIG_ICG_SD_MISO_GPIO < 0 || CONFIG_ICG_SD_CS_GPIO < 0) {
        ESP_LOGW(TAG, "SD disabled: set the four display-board SD GPIOs in menuconfig -> ICG prototype");
        return ESP_ERR_NOT_SUPPORTED;
    }

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = CONFIG_ICG_SD_MOSI_GPIO,
        .miso_io_num = CONFIG_ICG_SD_MISO_GPIO,
        .sclk_io_num = CONFIG_ICG_SD_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64 * 1024,
    };
    esp_err_t ret = spi_bus_initialize(ICG_SD_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SD SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = ICG_SD_HOST;
    host.max_freq_khz = 10000;

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = CONFIG_ICG_SD_CS_GPIO;
    slot.host_id = ICG_SD_HOST;

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 32 * 1024,
    };

    ret = esp_vfs_fat_sdspi_mount(MOUNT, &host, &slot, &mount_cfg, &s_card);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SD mount failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_ready = true;
    sdmmc_card_print_info(stdout, s_card);
    return ESP_OK;
}

bool storage_ready(void)
{
    return s_ready;
}

const char *storage_mount_point(void)
{
    return MOUNT;
}
