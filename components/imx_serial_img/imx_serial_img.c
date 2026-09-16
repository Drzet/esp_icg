/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart_vfs.h"
#include "imx_serial_img.h"

static const char *TAG = "imx_serial_img";

#define CHUNK_BYTES     4096
#define YIELD_EVERY     32

esp_err_t imx_serial_send_blob(const char *name, const char *fmt,
                               uint32_t w, uint32_t h,
                               const uint8_t *data, size_t len,
                               const char *extra)
{
    uint32_t crc = esp_rom_crc32_le(0, data, len);

    uart_vfs_dev_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_LF);
    esp_log_level_t prev = esp_log_level_get("*");
    esp_log_level_set("*", ESP_LOG_NONE);

    printf("\nIMGSTART name=%s fmt=%s w=%" PRIu32 " h=%" PRIu32 " len=%u crc32=%08" PRIx32 "%s%s\n",
           name, fmt, w, h, (unsigned)len, crc,
           extra ? " " : "", extra ? extra : "");
    fflush(stdout);

    size_t sent = 0;
    unsigned chunks = 0;
    while (sent < len) {
        size_t n = len - sent;
        if (n > CHUNK_BYTES) {
            n = CHUNK_BYTES;
        }
        size_t wrote = fwrite(data + sent, 1, n, stdout);
        if (wrote != n) {
            break;
        }
        sent += wrote;
        if ((++chunks % YIELD_EVERY) == 0) {
            vTaskDelay(1);
        }
    }
    fflush(stdout);
    printf("\nIMGEND\n");
    fflush(stdout);

    esp_log_level_set("*", prev);
    uart_vfs_dev_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);

    if (sent != len) {
        ESP_LOGE(TAG, "%s: sent %u of %u bytes", name, (unsigned)sent, (unsigned)len);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "sent %s (%s, %u bytes, crc32=%08" PRIx32 ")", name, fmt, (unsigned)len, crc);
    return ESP_OK;
}
