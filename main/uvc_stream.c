#include "uvc_stream.h"
#include <inttypes.h>
#include <stdlib.h>
#include "driver/jpeg_encode.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "tusb.h"
#include "usb_device_uvc.h"

/* One encoder buffer plus the component's independent USB transfer buffer:
 * while USB sends A, the encoder can produce B. Never overwrite either owner.
 * Use the former encoder capacity rather than assume typical JPEG sizes. */
#define JPEG_CAPACITY (ICG_UVC_WIDTH * ICG_UVC_HEIGHT * 2u)
#define HOST_IDLE_US 1000000
#define STATS_US 2000000

typedef enum { SLOT_FREE, SLOT_ENCODING, SLOT_READY, SLOT_USB } slot_state_t;
static const char *TAG = "uvc_stream";
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static slot_state_t s_slot;
static bool s_started, s_disabled;
static uint32_t s_generation;
static int64_t s_last_request;
static SemaphoreHandle_t s_ready;
static jpeg_encoder_handle_t s_encoder;
static uint8_t *s_jpeg, *s_transfer;
static size_t s_capacity;
static uvc_fb_t s_frame;
static uint32_t s_delivered;
/* Statistics below are owned by the capture task. */
static uint32_t s_encoded, s_skipped, s_errors, s_last_delivered;
static uint64_t s_bytes;
static int64_t s_encode_us, s_stats_start;

static bool host_present(void)
{
    return tud_mounted() && !tud_suspended() && tud_video_n_streaming(0, 0);
}

bool uvc_stream_active(void)
{
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_lock);
    bool active = s_started && !s_disabled && now - s_last_request < HOST_IDLE_US;
    portEXIT_CRITICAL(&s_lock);
    return active && host_present();
}

void uvc_stream_discard_ready(void)
{
    portENTER_CRITICAL(&s_lock);
    ++s_generation;
    if (s_slot == SLOT_READY) s_slot = SLOT_FREE;
    /* ENCODING and USB remain owned until their respective completion paths. */
    portEXIT_CRITICAL(&s_lock);
}

void uvc_stream_disable(void)
{
    portENTER_CRITICAL(&s_lock);
    s_disabled = true;
    s_started = false;
    ++s_generation;
    if (s_slot == SLOT_READY) s_slot = SLOT_FREE;
    portEXIT_CRITICAL(&s_lock);
}

static esp_err_t start_cb(uvc_format_t format, int width, int height, int rate, void *ctx)
{
    (void)ctx;
    if (format != UVC_FORMAT_JPEG || width != ICG_UVC_WIDTH ||
        height != ICG_UVC_HEIGHT || rate != CONFIG_UVC_CAM1_FRAMERATE) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_lock);
    bool disabled = s_disabled;
    if (!disabled) {
        ++s_generation;
        if (s_slot == SLOT_READY) s_slot = SLOT_FREE;
        s_started = true;
        s_last_request = now;
    }
    portEXIT_CRITICAL(&s_lock);
    if (disabled) return ESP_ERR_INVALID_STATE;
    ESP_LOGI(TAG, "host opened %dx%d MJPEG, descriptor=%d fps", width, height, rate);
    return ESP_OK;
}

static void stop_cb(void *ctx)
{
    (void)ctx;
    portENTER_CRITICAL(&s_lock);
    s_started = false;
    ++s_generation;
    if (s_slot == SLOT_READY) s_slot = SLOT_FREE;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "host stopped/suspended stream");
}

static uvc_fb_t *get_cb(void *ctx)
{
    (void)ctx;
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_lock);
    bool active = s_started && !s_disabled;
    uint32_t generation = s_generation;
    if (active) s_last_request = now;
    portEXIT_CRITICAL(&s_lock);
    if (!active || !host_present()) return NULL;
    /* Bounded wait: never hold TinyUSB's event task or wait indefinitely when
     * the camera fails/disconnects. This callback runs in the UVC video task. */
    if (xSemaphoreTake(s_ready, pdMS_TO_TICKS(100)) != pdTRUE) return NULL;
    bool present = host_present();
    portENTER_CRITICAL(&s_lock);
    bool ready = present && s_started && !s_disabled &&
                 s_generation == generation && s_slot == SLOT_READY;
    if (ready) {
        s_slot = SLOT_USB;
        ++s_delivered;
    }
    bool retry = !ready && s_started && !s_disabled && s_slot == SLOT_READY;
    portEXIT_CRITICAL(&s_lock);
    /* A renegotiation may have replaced the signalled frame while we waited. */
    if (retry) xSemaphoreGive(s_ready);
    return ready ? &s_frame : NULL;
}

static void return_cb(uvc_fb_t *fb, void *ctx)
{
    (void)ctx;
    if (fb != &s_frame) return;
    /* usb_device_uvc has copied the JPEG before returning it. Only now may
     * the next encode write this buffer while USB sends its separate copy. */
    portENTER_CRITICAL(&s_lock);
    if (s_slot == SLOT_USB) s_slot = SLOT_FREE;
    portEXIT_CRITICAL(&s_lock);
}

static void log_stats(void)
{
    int64_t now = esp_timer_get_time();
    if (!s_stats_start) s_stats_start = now;
    int64_t elapsed = now - s_stats_start;
    if (elapsed < STATS_US) return;
    portENTER_CRITICAL(&s_lock);
    uint32_t delivered = s_delivered;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "encode=%.2f fps USB_handoff=%.2f fps jpeg_avg=%" PRId64
             " us bytes_avg=%" PRIu64 " skipped_before_encode=%" PRIu32 " errors=%" PRIu32,
             (double)s_encoded * 1000000 / elapsed,
             (double)(delivered - s_last_delivered) * 1000000 / elapsed,
             s_encoded ? s_encode_us / s_encoded : 0,
             s_encoded ? s_bytes / s_encoded : 0, s_skipped, s_errors);
    s_encoded = s_skipped = s_errors = 0;
    s_encode_us = 0;
    s_bytes = 0;
    s_last_delivered = delivered;
    s_stats_start = now;
}

void uvc_stream_submit(const uint8_t *rgb565, size_t len)
{
    log_stats();
    if (!rgb565 || len < JPEG_CAPACITY || !uvc_stream_active()) return;
    portENTER_CRITICAL(&s_lock);
    bool room = s_started && !s_disabled && s_slot == SLOT_FREE;
    uint32_t generation = s_generation;
    if (room) s_slot = SLOT_ENCODING;
    portEXIT_CRITICAL(&s_lock);
    if (!room) { ++s_skipped; return; }

    const jpeg_encode_cfg_t cfg = {
        .width = ICG_UVC_WIDTH, .height = ICG_UVC_HEIGHT,
        .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample = JPEG_DOWN_SAMPLING_YUV422,
        .image_quality = ICG_UVC_QUALITY,
    };
    uint32_t size = 0;
    int64_t begin = esp_timer_get_time();
    esp_err_t ret = jpeg_encoder_process(s_encoder, &cfg, rgb565, JPEG_CAPACITY,
                                         s_jpeg, s_capacity, &size);
    int64_t duration = esp_timer_get_time() - begin;
    bool valid = ret == ESP_OK && size > 0 && size <= JPEG_CAPACITY;
    if (valid) {
        ++s_encoded;
        s_encode_us += duration;
        s_bytes += size;
    } else {
        ++s_errors;
        ESP_LOGE(TAG, "JPEG failed: %s size=%" PRIu32, esp_err_to_name(ret), size);
    }
    bool active = uvc_stream_active();
    portENTER_CRITICAL(&s_lock);
    bool publish = valid && active && s_started && !s_disabled && generation == s_generation;
    if (publish) {
        s_frame.len = size;
        s_frame.timestamp.tv_sec = begin / 1000000;
        s_frame.timestamp.tv_usec = begin % 1000000;
        s_slot = SLOT_READY;
    } else {
        s_slot = SLOT_FREE;
    }
    portEXIT_CRITICAL(&s_lock);
    if (publish) xSemaphoreGive(s_ready);
}

esp_err_t uvc_stream_init(void)
{
    jpeg_encode_memory_alloc_cfg_t out = { .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER };
    s_jpeg = jpeg_alloc_encoder_mem(JPEG_CAPACITY, &out, &s_capacity);
    s_transfer = heap_caps_malloc(JPEG_CAPACITY, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_ready = xSemaphoreCreateBinary();
    esp_err_t ret = ESP_ERR_NO_MEM;
    if (!s_jpeg || !s_transfer || !s_ready) goto fail;
    jpeg_encode_engine_cfg_t engine = { .timeout_ms = 1000 };
    ret = jpeg_new_encoder_engine(&engine, &s_encoder);
    if (ret != ESP_OK) goto fail;
    s_frame = (uvc_fb_t) {
        .buf = s_jpeg, .width = ICG_UVC_WIDTH, .height = ICG_UVC_HEIGHT,
        .format = UVC_FORMAT_JPEG,
    };
    uvc_device_config_t config = {
        .uvc_buffer = s_transfer, .uvc_buffer_size = JPEG_CAPACITY,
        .start_cb = start_cb, .stop_cb = stop_cb,
        .fb_get_cb = get_cb, .fb_return_cb = return_cb,
    };
    ret = uvc_device_config(0, &config);
    if (ret != ESP_OK) goto fail;
    ret = uvc_device_init();
    if (ret != ESP_OK) {
        /* A partially initialized component may still own callbacks/buffers.
         * Disable production but retain their storage until reboot. */
        uvc_stream_disable();
        return ret;
    }
    ESP_LOGI(TAG, "HS bulk MJPEG q%d; encoder/USB buffers=%u bytes each",
             ICG_UVC_QUALITY, (unsigned)JPEG_CAPACITY);
    return ESP_OK;
fail:
    if (s_encoder) { jpeg_del_encoder_engine(s_encoder); s_encoder = NULL; }
    if (s_ready) { vSemaphoreDelete(s_ready); s_ready = NULL; }
    free(s_jpeg);
    free(s_transfer);
    s_jpeg = s_transfer = NULL;
    return ret;
}
