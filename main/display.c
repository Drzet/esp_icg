#include "display.h"

#include <stdlib.h>
#include <string.h>
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ili9488.h"
#include "app_config.h"

static const char *TAG = "display";
static esp_lcd_panel_handle_t s_panel;
static uint16_t *s_ui_line;

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

static esp_err_t fill_rect(int x0, int y0, int x1, int y1, uint16_t color)
{
    if (!s_panel || !s_ui_line || x0 < 0 || y0 < 0 || x1 > ICG_LCD_WIDTH || y1 > ICG_LCD_HEIGHT || x1 <= x0 || y1 <= y0) {
        return ESP_ERR_INVALID_ARG;
    }
    const int w = x1 - x0;
    for (int x = 0; x < w; ++x) s_ui_line[x] = color;
    for (int y = y0; y < y1; ++y) {
        esp_err_t ret = esp_lcd_panel_draw_bitmap(s_panel, x0, y, x1, y + 1, s_ui_line);
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}

static esp_err_t draw_start_icon(void)
{
    const uint16_t c = rgb565(30, 220, 70);
    for (int row = 0; row < 24; ++row) {
        int half = row < 12 ? row : 23 - row;
        int x0 = 67;
        int x1 = 67 + half * 2 + 2;
        ESP_RETURN_ON_ERROR(fill_rect(x0, 288 + row, x1, 289 + row, c), TAG, "start icon");
    }
    return ESP_OK;
}

static esp_err_t draw_stop_icon(void)
{
    return fill_rect(228, 289, 252, 313, rgb565(235, 70, 60));
}

static esp_err_t draw_record_icon(bool active)
{
    const uint16_t c = active ? rgb565(255, 20, 20) : rgb565(150, 35, 35);
    for (int row = -12; row <= 12; ++row) {
        int width = 24 - abs(row) * 2;
        if (width < 2) width = 2;
        ESP_RETURN_ON_ERROR(fill_rect(400 - width / 2, 301 + row, 400 + width / 2, 302 + row, c), TAG, "record icon");
    }
    return ESP_OK;
}

esp_err_t display_init(void)
{
    spi_bus_config_t buscfg = {
        .sclk_io_num = ICG_LCD_PIN_SCLK,
        .mosi_io_num = ICG_LCD_PIN_MOSI,
        .miso_io_num = ICG_LCD_PIN_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = ICG_LCD_WIDTH * 3 * 16,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(ICG_LCD_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "spi bus");

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = ICG_LCD_PIN_DC,
        .cs_gpio_num = ICG_LCD_PIN_CS,
        .pclk_hz = 20 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)ICG_LCD_HOST, &io_cfg, &io), TAG, "lcd io");

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = ICG_LCD_PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 18,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_ili9488_ips(io, &panel_cfg, ICG_LCD_WIDTH, &s_panel), TAG, "ili9488");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "lcd reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "lcd init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "lcd on");

    s_ui_line = heap_caps_malloc(ICG_LCD_WIDTH * sizeof(uint16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_ui_line) return ESP_ERR_NO_MEM;

    ESP_RETURN_ON_ERROR(fill_rect(0, 0, ICG_LCD_WIDTH, ICG_LCD_HEIGHT, 0x0000), TAG, "clear");
    return display_draw_ui(false, false, false);
}

esp_err_t display_draw_preview(const uint16_t *rgb565_frame)
{
    if (!s_panel || !rgb565_frame) return ESP_ERR_INVALID_ARG;
    return esp_lcd_panel_draw_bitmap(s_panel, 0, 0, ICG_LCD_WIDTH, ICG_PREVIEW_HEIGHT, rgb565_frame);
}

esp_err_t display_draw_ui(bool camera_running, bool recording, bool sd_ready)
{
    const uint16_t bg = rgb565(18, 18, 18);
    const uint16_t edge = rgb565(60, 60, 60);
    ESP_RETURN_ON_ERROR(fill_rect(0, ICG_PREVIEW_HEIGHT, ICG_LCD_WIDTH, ICG_LCD_HEIGHT, bg), TAG, "ui bg");
    ESP_RETURN_ON_ERROR(fill_rect(159, ICG_PREVIEW_HEIGHT, 161, ICG_LCD_HEIGHT, edge), TAG, "divider");
    ESP_RETURN_ON_ERROR(fill_rect(319, ICG_PREVIEW_HEIGHT, 321, ICG_LCD_HEIGHT, edge), TAG, "divider");
    ESP_RETURN_ON_ERROR(draw_start_icon(), TAG, "start");
    ESP_RETURN_ON_ERROR(draw_stop_icon(), TAG, "stop");
    ESP_RETURN_ON_ERROR(draw_record_icon(recording), TAG, "record");

    if (camera_running) ESP_RETURN_ON_ERROR(fill_rect(4, 283, 28, 286, rgb565(40, 220, 60)), TAG, "camera status");
    if (sd_ready) ESP_RETURN_ON_ERROR(fill_rect(452, 283, 476, 286, rgb565(60, 120, 255)), TAG, "sd status");
    return ESP_OK;
}
