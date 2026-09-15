#include "recorder.h"

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "driver/jpeg_encode.h"
#include "storage.h"

#define RECORDER_SLOTS 2

static const char *TAG = "recorder";

typedef struct {
    uint8_t *buf;
    size_t capacity;
    size_t len;
    uint32_t width;
    uint32_t height;
} frame_slot_t;

static frame_slot_t s_slots[RECORDER_SLOTS];
static QueueHandle_t s_free_q;
static QueueHandle_t s_pending_q;
static jpeg_encoder_handle_t s_jpeg;
static uint8_t *s_jpeg_buf;
static size_t s_jpeg_capacity;
static volatile bool s_active;
static volatile uint32_t s_dropped;
static uint32_t s_frame_number;

static void recorder_task(void *arg)
{
    (void)arg;
    int idx;
    while (1) {
        if (xQueueReceive(s_pending_q, &idx, portMAX_DELAY) != pdTRUE) continue;

        frame_slot_t *slot = &s_slots[idx];
        if (s_active && storage_ready()) {
            jpeg_encode_cfg_t cfg = {
                .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
                .sub_sample = JPEG_DOWN_SAMPLING_YUV422,
                .image_quality = CONFIG_ICG_JPEG_QUALITY,
                .width = slot->width,
                .height = slot->height,
                .pixel_reverse = false,
            };

            uint32_t jpeg_size = 0;
            esp_err_t ret = jpeg_encoder_process(s_jpeg, &cfg,
                                                 slot->buf, slot->len,
                                                 s_jpeg_buf, s_jpeg_capacity,
                                                 &jpeg_size);
            if (ret == ESP_OK) {
                char path[96];
                snprintf(path, sizeof(path), "%s/ICG_%06lu.jpg",
                         storage_mount_point(), (unsigned long)s_frame_number++);
                FILE *f = fopen(path, "wb");
                if (f) {
                    size_t written = fwrite(s_jpeg_buf, 1, jpeg_size, f);
                    fclose(f);
                    if (written != jpeg_size) {
                        ESP_LOGW(TAG, "Short SD write: %u/%u", (unsigned)written, (unsigned)jpeg_size);
                    }
                } else {
                    ESP_LOGW(TAG, "Failed to create %s", path);
                }
            } else {
                ESP_LOGW(TAG, "JPEG encode failed: %s", esp_err_to_name(ret));
            }
        }

        xQueueSend(s_free_q, &idx, portMAX_DELAY);
    }
}

esp_err_t recorder_init(void)
{
    const size_t max_raw = (size_t)CONFIG_ICG_RECORD_MAX_WIDTH * CONFIG_ICG_RECORD_MAX_HEIGHT * 2;
    for (int i = 0; i < RECORDER_SLOTS; ++i) {
        s_slots[i].capacity = max_raw;
        s_slots[i].buf = heap_caps_malloc(max_raw, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_slots[i].buf) return ESP_ERR_NO_MEM;
    }

    jpeg_encode_memory_alloc_cfg_t mem_cfg = {
        .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
    };
    size_t actual = 0;
    s_jpeg_buf = jpeg_alloc_encoder_mem(max_raw / 2, &mem_cfg, &actual);
    if (!s_jpeg_buf) return ESP_ERR_NO_MEM;
    s_jpeg_capacity = actual;

    jpeg_encode_engine_cfg_t engine_cfg = {
        .intr_priority = 0,
        .timeout_ms = -1,
    };
    ESP_RETURN_ON_ERROR(jpeg_new_encoder_engine(&engine_cfg, &s_jpeg), TAG, "JPEG engine");

    s_free_q = xQueueCreate(RECORDER_SLOTS, sizeof(int));
    s_pending_q = xQueueCreate(RECORDER_SLOTS, sizeof(int));
    if (!s_free_q || !s_pending_q) return ESP_ERR_NO_MEM;

    for (int i = 0; i < RECORDER_SLOTS; ++i) xQueueSend(s_free_q, &i, 0);
    if (xTaskCreate(recorder_task, "recorder", 6144, NULL, 5, NULL) != pdPASS) return ESP_ERR_NO_MEM;
    return ESP_OK;
}

void recorder_set_active(bool active)
{
    if (active && !storage_ready()) {
        s_active = false;
        return;
    }
    s_active = active;
    if (active) {
        s_frame_number = 0;
        s_dropped = 0;
    }
}

bool recorder_active(void)
{
    return s_active;
}

uint32_t recorder_dropped_frames(void)
{
    return s_dropped;
}

void recorder_submit_rgb565(const uint8_t *buf, uint32_t width, uint32_t height, size_t len)
{
    if (!s_active || !buf || !storage_ready()) return;

    int idx;
    if (xQueueReceive(s_free_q, &idx, 0) != pdTRUE) {
        ++s_dropped;
        return;
    }

    frame_slot_t *slot = &s_slots[idx];
    if (len > slot->capacity) {
        ++s_dropped;
        xQueueSend(s_free_q, &idx, 0);
        return;
    }

    memcpy(slot->buf, buf, len);
    slot->len = len;
    slot->width = width;
    slot->height = height;
    if (xQueueSend(s_pending_q, &idx, 0) != pdTRUE) {
        ++s_dropped;
        xQueueSend(s_free_q, &idx, 0);
    }
}
