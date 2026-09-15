#include "display.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ili9488.h"
#include "app_config.h"

static const char *TAG = "display";
static esp_lcd_panel_handle_t s_panel;
static SemaphoreHandle_t s_lcd_mutex;
static SemaphoreHandle_t s_lcd_done;
static uint16_t *s_line;

static bool lcd_done_cb(esp_lcd_panel_io_handle_t io,
                        esp_lcd_panel_io_event_data_t *edata,
                        void *user_ctx)
{
    (void)io;
    (void)edata;
    (void)user_ctx;
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(s_lcd_done, &hp);
    return hp == pdTRUE;
}

static esp_err_t draw_line_sync(int y, const uint16_t *line)
{
    if (xSemaphoreTake(s_lcd_mutex, portMAX_DELAY) != pdTRUE) return ESP_ERR_TIMEOUT;
    while (xSemaphoreTake(s_lcd_done, 0) == pdTRUE) {}

    esp_err_t ret = esp_lcd_panel_draw_bitmap(s_panel, 0, y, ICG_LCD_WIDTH, y + 1, line);
    if (ret == ESP_OK && xSemaphoreTake(s_lcd_done, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ret = ESP_ERR_TIMEOUT;
    }

    xSemaphoreGive(s_lcd_mutex);
    return ret;
}

static esp_err_t clear_screen(void)
{
    for (int x = 0; x < ICG_LCD_WIDTH; ++x) s_line[x] = 0;
    for (int y = 0; y < ICG_LCD_HEIGHT; ++y) {
        ESP_RETURN_ON_ERROR(draw_line_sync(y, s_line), TAG, "clear line");
    }
    return ESP_OK;
}

esp_err_t display_init(void)
{
    s_lcd_mutex = xSemaphoreCreateMutex();
    s_lcd_done = xSemaphoreCreateBinary();
    if (!s_lcd_mutex || !s_lcd_done) return ESP_ERR_NO_MEM;

    spi_bus_config_t buscfg = {
        .sclk_io_num = ICG_LCD_PIN_SCLK,
        .mosi_io_num = ICG_LCD_PIN_MOSI,
        .miso_io_num = ICG_LCD_PIN_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = ICG_LCD_WIDTH * 3,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(ICG_LCD_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "spi bus");

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = ICG_LCD_PIN_DC,
        .cs_gpio_num = ICG_LCD_PIN_CS,
        .pclk_hz = 40 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 1,
        .on_color_trans_done = lcd_done_cb,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)ICG_LCD_HOST,
                                                  &io_cfg, &io), TAG, "lcd io");

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = ICG_LCD_PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 18,
    };

    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_ili9488_ips(io, &panel_cfg, ICG_LCD_WIDTH, &s_panel),
                        TAG, "ili9488");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(s_panel, true), TAG, "landscape");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(s_panel, false), TAG, "normal colors");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "display on");

    s_line = heap_caps_malloc(ICG_LCD_WIDTH * sizeof(uint16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_line) return ESP_ERR_NO_MEM;

    return clear_screen();
}

esp_err_t display_draw_preview(const uint16_t *rgb565_frame)
{
    if (!rgb565_frame) return ESP_ERR_INVALID_ARG;

    for (int y = 0; y < ICG_PREVIEW_HEIGHT; ++y) {
        const uint16_t *line = rgb565_frame + (size_t)y * ICG_LCD_WIDTH;
        esp_err_t ret = draw_line_sync(y, line);
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}
