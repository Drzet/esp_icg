#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "wt_bsp.h"
#include "camera.h"
#include "display.h"
#include "recorder.h"
#include "storage.h"
#include "touch.h"
#include "app_config.h"

static const char *TAG = "icg";

static void refresh_ui(void)
{
    display_draw_ui(camera_running(), recorder_active(), storage_ready());
}

static void ui_task(void *arg)
{
    (void)arg;
    bool was_pressed = false;

    while (1) {
        uint16_t x = 0, y = 0;
        bool pressed = touch_read(&x, &y);

        if (pressed && !was_pressed && y >= ICG_PREVIEW_HEIGHT) {
            if (x < 160) {
                esp_err_t ret = camera_start();
                if (ret != ESP_OK) ESP_LOGW(TAG, "START failed: %s", esp_err_to_name(ret));
            } else if (x < 320) {
                esp_err_t ret = camera_stop();
                if (ret != ESP_OK) ESP_LOGW(TAG, "STOP failed: %s", esp_err_to_name(ret));
            } else {
                if (camera_running() && storage_ready()) {
                    recorder_set_active(!recorder_active());
                }
            }
            refresh_ui();
        }

        was_pressed = pressed;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(display_init());
    ESP_ERROR_CHECK(touch_init());

    esp_err_t sd_ret = storage_init();
    if (sd_ret != ESP_OK && sd_ret != ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "SD unavailable: %s", esp_err_to_name(sd_ret));
    }

    ESP_ERROR_CHECK(recorder_init());

    ESP_LOGI(TAG, "Initializing WT9932P4-TINY BSP / CSI");
    ESP_ERROR_CHECK(wt_bsp_init());
    ESP_ERROR_CHECK(camera_init());

    refresh_ui();
    xTaskCreate(ui_task, "ui", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "Ready: left=START, middle=STOP, right=REC");
}
