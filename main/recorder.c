#include "recorder.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "app_config.h"
#include "avi_writer.h"
#include "shared_spi.h"
#include "driver/jpeg_encode.h"
#include "driver/sdspi_host.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "sd_protocol_defs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define MOUNT_POINT "/sdcard"
#define INDEX_CAPACITY 6000
#define JPEG_QUEUE_DEPTH 4
#define JPEG_QUEUE_BYTES (3u * 1024u * 1024u)
#define PERIOD_US (1000000 / ICG_RECORD_FPS)
#define FILE_BUFFER_BYTES (512u * 1024u)
#define DMA_BUFFER_BYTES (16u * 1024u)
#define PREALLOC_SECONDS 30u
#define PREALLOC_BYTES (64u * 1024u * 1024u)

static const char *TAG = "recorder";
typedef enum { REC_IDLE, REC_STARTING, REC_ACTIVE, REC_STOPPING } rec_state_t;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static rec_state_t s_state;
static bool s_cancel_start, s_busy, s_ready;
static uint32_t s_dropped;
static int64_t s_frame_us, s_next_us;
static TaskHandle_t s_task, s_writer_task;
typedef struct {
    uint8_t *data;
    uint32_t size;
    int64_t time_us, copy_us, encode_us;
} jpeg_frame_t;
static QueueHandle_t s_jpeg_queue;
/* Includes the frame being written; protected by s_lock. */
static size_t s_queued_bytes;
static uint32_t s_queue_drops;
static int64_t s_copy_us;
static uint32_t s_width, s_height;
static uint8_t *s_raw, *s_jpeg, *s_file_buffer, *s_dma_buffer;
static size_t s_raw_size, s_jpeg_size;
static jpeg_encoder_handle_t s_encoder;
static avi_index_t *s_index;
/* After initialization, only writer_task accesses card, AVI and files. */
static sdmmc_card_t *s_card;
static avi_writer_t s_avi = { .fd = -1 };
static char s_path[48];
/* Mount/init and recording I/O each have a single owner. No per-sector logging
 * or cross-task lock is needed in this callback. Times include driver waiting
 * and scheduling, not just time with SCLK active. */
static bool s_measure_sd;
static struct {
    uint32_t single, multi, errors;
    uint64_t bytes;
    int64_t time_us, max_us;
} s_sd_stats;

static esp_err_t measured_sd_transaction(int slot, sdmmc_command_t *cmd)
{
    bool measure = s_measure_sd &&
        (cmd->opcode == MMC_WRITE_BLOCK_SINGLE || cmd->opcode == MMC_WRITE_BLOCK_MULTIPLE);
    int64_t begin = measure ? esp_timer_get_time() : 0;
    esp_err_t ret = sdspi_host_do_transaction(slot, cmd);
    if (measure) {
        int64_t elapsed = esp_timer_get_time() - begin;
        s_sd_stats.time_us += elapsed;
        if (elapsed > s_sd_stats.max_us) s_sd_stats.max_us = elapsed;
        if (cmd->opcode == MMC_WRITE_BLOCK_SINGLE) ++s_sd_stats.single;
        else ++s_sd_stats.multi;
        if (ret != ESP_OK || cmd->error != ESP_OK) ++s_sd_stats.errors;
        else s_sd_stats.bytes += cmd->datalen;
    }
    return ret;
}

static void log_sd_stats(void)
{
    ESP_LOGI(TAG, "SD I/O cumulative: CMD24=%" PRIu32 " CMD25=%" PRIu32
             " KiB=%" PRIu64 " driver_ms=%" PRId64 " KiB/s=%" PRIu64
             " max_cmd_us=%" PRId64 " errors=%" PRIu32,
             s_sd_stats.single, s_sd_stats.multi, s_sd_stats.bytes / 1024,
             s_sd_stats.time_us / 1000,
             s_sd_stats.time_us > 0 ? s_sd_stats.bytes * 1000000 / 1024 / s_sd_stats.time_us : 0,
             s_sd_stats.max_us, s_sd_stats.errors);
}

static void unmount_card(void)
{
    if (s_card) {
        shared_spi_mount_lock();
        esp_err_t err = esp_vfs_fat_sdcard_unmount(MOUNT_POINT, s_card);
        shared_spi_unlock();
        if (err != ESP_OK) ESP_LOGE(TAG, "unmount failed: %s", esp_err_to_name(err));
        s_card = NULL;
    }
}

static bool mount_card(void)
{
    if (s_card) return true;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = ICG_TOUCH_HOST;
    host.max_freq_khz = ICG_SD_CLOCK_KHZ;
    host.unaligned_multi_block_rw_max_chunk_size = DMA_BUFFER_BYTES / 512u;
    host.do_transaction = measured_sd_transaction;
    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.host_id = ICG_TOUCH_HOST;
    slot.gpio_cs = ICG_SD_PIN_CS;
    esp_vfs_fat_sdmmc_mount_config_t mount = {
        .format_if_mount_failed = false,
        .max_files = 2,
        .allocation_unit_size = 32 * 1024,
    };
    ESP_LOGI(TAG, "SD mount starting: requested maximum %d kHz", ICG_SD_CLOCK_KHZ);
    shared_spi_mount_lock();
    esp_err_t ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot, &mount, &s_card);
    /* A failed mount cleans up its temporary card/device. Retry initialization
     * from scratch, still excluding touch, when high-speed negotiation fails. */
    if (host.max_freq_khz > SDMMC_FREQ_DEFAULT &&
        (ret == ESP_ERR_INVALID_RESPONSE || ret == ESP_ERR_INVALID_CRC ||
         ret == ESP_ERR_NOT_SUPPORTED)) {
        ESP_LOGW(TAG, "SD init at max %d kHz failed (%s); retrying at %d kHz",
                 host.max_freq_khz, esp_err_to_name(ret), SDMMC_FREQ_DEFAULT);
        s_card = NULL;
        host.max_freq_khz = SDMMC_FREQ_DEFAULT;
        ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot, &mount, &s_card);
    }
    shared_spi_unlock();
    if (ret != ESP_OK) {
        s_card = NULL;
        ESP_LOGE(TAG, "SD initialization/mount failed: %s; card was not formatted",
                 esp_err_to_name(ret));
        return false;
    }
    sdmmc_card_print_info(stdout, s_card);
    int actual_khz = 0;
    if (sdspi_host_get_real_freq(s_card->host.slot, &actual_khz) == ESP_OK) {
        ESP_LOGI(TAG, "SD clock: actual=%d kHz requested_max=%d kHz; sector=%d bytes",
                 actual_khz, ICG_SD_CLOCK_KHZ, s_card->csd.sector_size);
    }
    return true;
}

static bool start_file(void)
{
    if (!mount_card()) return false;
    /* 8.3 filenames work without enabling FatFs long names; never overwrite. */
    int fd = -1;
    for (unsigned n = 1; n <= 99999; ++n) {
        snprintf(s_path, sizeof(s_path), MOUNT_POINT "/ICG%05u.AVI", n);
        fd = open(s_path, O_CREAT | O_EXCL | O_RDWR, 0666);
        if (fd >= 0 || errno != EEXIST) break;
    }
    if (fd < 0) {
        ESP_LOGE(TAG, "cannot create recording: errno=%d", errno);
        unmount_card();
        return false;
    }
    /* Reserve the name exclusively, then close it before the FAT helper opens
     * it. This writer is the only task that creates/deletes recording files.
     * Growing with VFS ftruncate() zero-fills all 64 MiB on ESP-IDF, which can
     * block recording start for minutes. f_expand() only allocates clusters. */
    if (close(fd) != 0) {
        ESP_LOGE(TAG, "cannot close new recording: errno=%d", errno);
        unlink(s_path);
        unmount_card();
        return false;
    }
    ESP_LOGI(TAG, "preallocating %u MiB for %s", PREALLOC_BYTES / (1024u * 1024u), s_path);
    int64_t alloc_begin = esp_timer_get_time();
    if (esp_vfs_fat_create_contiguous_file(MOUNT_POINT, s_path, PREALLOC_BYTES, true) != ESP_OK) {
        ESP_LOGE(TAG, "AVI allocation failed: errno=%d (requires 64 MiB contiguous free space)", errno);
        unlink(s_path);
        unmount_card();
        return false;
    }
    ESP_LOGI(TAG, "preallocation complete in %" PRId64 " ms", (esp_timer_get_time() - alloc_begin) / 1000);
    /* No O_TRUNC: preserve the allocated FAT chain. */
    fd = open(s_path, O_RDWR);
    if (fd < 0) {
        ESP_LOGE(TAG, "cannot reopen allocated AVI: errno=%d", errno);
        unlink(s_path);
        unmount_card();
        return false;
    }
    if (!avi_begin(&s_avi, fd, s_index, INDEX_CAPACITY, s_width, s_height, PERIOD_US,
                   s_file_buffer, FILE_BUFFER_BYTES, s_dma_buffer, DMA_BUFFER_BYTES)) {
        close(fd);
        s_avi.fd = -1;
        unlink(s_path);
        unmount_card();
        return false;
    }
    ESP_LOGI(TAG, "RECORD %s: %" PRIu32 "x%" PRIu32
             " MJPEG q%d, up to %d fps, buffer=%u KiB DMA=%u KiB prealloc=%u MiB (~%u s)",
             s_path, s_width, s_height, ICG_RECORD_QUALITY, ICG_RECORD_FPS,
             FILE_BUFFER_BYTES / 1024u, DMA_BUFFER_BYTES / 1024u,
             PREALLOC_BYTES / (1024u * 1024u), PREALLOC_SECONDS);
    memset(&s_sd_stats, 0, sizeof(s_sd_stats));
    s_measure_sd = true;
    return true;
}

static void finish_file(void)
{
    bool ok = true;
    if (s_avi.fd >= 0) {
        ok = avi_finish(&s_avi);
        /* avi_finish() flushes our buffer and writes the final index/header. Remove
         * unused preallocated space, then perform the only fsync of recording. */
        off_t final_size = (off_t)s_avi.end + 8 + (off_t)s_avi.frames * 16;
        if (ok && ftruncate(s_avi.fd, final_size) != 0) ok = false;
        if (ok && fsync(s_avi.fd) != 0) ok = false;
        if (close(s_avi.fd) != 0) ok = false;
        s_avi.fd = -1;
        if (s_avi.frames == 0) unlink(s_path);
    }
    uint32_t dropped, queue_drops;
    portENTER_CRITICAL(&s_lock);
    dropped = s_dropped;
    queue_drops = s_queue_drops;
    portEXIT_CRITICAL(&s_lock);
    unmount_card();
    log_sd_stats();
    s_measure_sd = false;
    if (ok) ESP_LOGI(TAG, "STOP finalized %s frames=%" PRIu32 " busy_drops=%" PRIu32 " queue_drops=%" PRIu32,
                    s_path, s_avi.frames, dropped, queue_drops);
    else ESP_LOGE(TAG, "STOP file finalization failed: %s (card full/removed or I/O error)", s_path);
}

static void request_error_stop(void)
{
    portENTER_CRITICAL(&s_lock);
    s_state = REC_STOPPING;
    portEXIT_CRITICAL(&s_lock);
    xTaskNotifyGive(s_writer_task);
}

static void release_frame(jpeg_frame_t *frame)
{
    free(frame->data);
    portENTER_CRITICAL(&s_lock);
    s_queued_bytes -= frame->size;
    portEXIT_CRITICAL(&s_lock);
}

static void writer_task(void *arg)
{
    (void)arg;
    bool failed = false;
    int64_t copy_total = 0, encode_total = 0, write_total = 0;
    uint64_t byte_total = 0;
    uint32_t samples = 0;
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        portENTER_CRITICAL(&s_lock);
        rec_state_t state = s_state;
        portEXIT_CRITICAL(&s_lock);
        if (state == REC_STARTING) {
            bool ok = start_file();
            failed = false;
            copy_total = encode_total = write_total = 0;
            byte_total = 0;
            samples = 0;
            portENTER_CRITICAL(&s_lock);
            s_next_us = 0;
            s_dropped = s_queue_drops = 0;
            s_state = ok ? (s_cancel_start ? REC_STOPPING : REC_ACTIVE) : REC_IDLE;
            portEXIT_CRITICAL(&s_lock);
        }
        jpeg_frame_t frame;
        while (xQueueReceive(s_jpeg_queue, &frame, 0) == pdTRUE) {
            if (!failed) {
                int64_t begin = esp_timer_get_time();
                bool ok = avi_frame(&s_avi, frame.data, frame.size, frame.time_us);
                write_total += esp_timer_get_time() - begin;
                copy_total += frame.copy_us;
                encode_total += frame.encode_us;
                byte_total += frame.size;
                ++samples;
                if (!ok) {
                    failed = true;
                    ESP_LOGE(TAG, "SD write or AVI limit: stopping and discarding unwritten queue (errno=%d)", errno);
                    request_error_stop();
                }
                if (samples == ICG_RECORD_FPS) {
                    uint32_t drops;
                    portENTER_CRITICAL(&s_lock);
                    drops = s_queue_drops;
                    portEXIT_CRITICAL(&s_lock);
                    ESP_LOGI(TAG, "record avg us: copy=%" PRId64 " jpeg=%" PRId64
                             " write=%" PRId64 " bytes=%" PRIu64 " queue_drops=%" PRIu32,
                             copy_total / samples, encode_total / samples,
                             write_total / samples, byte_total / samples, drops);
                    log_sd_stats();
                    copy_total = encode_total = write_total = 0;
                    byte_total = 0;
                    samples = 0;
                }
            }
            release_frame(&frame);
        }
        /* Snapshot producer completion BEFORE checking the queue. During STOP,
         * no new raw frame can enter. The producer enqueues before clearing busy,
         * so this ordering prevents closing a file ahead of its final JPEG. */
        portENTER_CRITICAL(&s_lock);
        bool drained_producer = s_state == REC_STOPPING && !s_busy;
        portEXIT_CRITICAL(&s_lock);
        if (drained_producer && uxQueueMessagesWaiting(s_jpeg_queue) == 0) {
            finish_file();
            portENTER_CRITICAL(&s_lock);
            s_state = REC_IDLE;
            portEXIT_CRITICAL(&s_lock);
        }
    }
}

static void recorder_task(void *arg)
{
    (void)arg;
    const jpeg_encode_cfg_t cfg = {
        .width = s_width, .height = s_height,
        .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample = JPEG_DOWN_SAMPLING_YUV422,
        .image_quality = ICG_RECORD_QUALITY,
    };
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        portENTER_CRITICAL(&s_lock);
        bool ready = s_ready;
        jpeg_frame_t frame = { .time_us = s_frame_us, .copy_us = s_copy_us };
        s_ready = false;
        portEXIT_CRITICAL(&s_lock);
        if (!ready) continue;

        int64_t begin = esp_timer_get_time();
        esp_err_t ret = jpeg_encoder_process(s_encoder, &cfg, s_raw, s_raw_size,
                                             s_jpeg, s_jpeg_size, &frame.size);
        frame.encode_us = esp_timer_get_time() - begin;
        if (ret == ESP_OK) {
            portENTER_CRITICAL(&s_lock);
            bool room = frame.size > 0 && frame.size <= JPEG_QUEUE_BYTES - s_queued_bytes;
            if (room) s_queued_bytes += frame.size;
            portEXIT_CRITICAL(&s_lock);
            bool queued = false;
            if (room) {
                /* Queue owns a private JPEG copy; encoder scratch can immediately
                 * be reused while the writer holds an earlier frame. */
                frame.data = heap_caps_malloc(frame.size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                if (frame.data) {
                    memcpy(frame.data, s_jpeg, frame.size);
                    queued = xQueueSend(s_jpeg_queue, &frame, 0) == pdTRUE;
                }
                if (!queued) release_frame(&frame);
            }
            if (!queued) {
                portENTER_CRITICAL(&s_lock);
                ++s_queue_drops;
                portEXIT_CRITICAL(&s_lock);
            }
        } else {
            ESP_LOGE(TAG, "JPEG encode failed: %s", esp_err_to_name(ret));
            request_error_stop();
        }
        portENTER_CRITICAL(&s_lock);
        s_busy = false;
        portEXIT_CRITICAL(&s_lock);
        xTaskNotifyGive(s_writer_task);
    }
}

esp_err_t recorder_init(uint32_t width, uint32_t height)
{
    ESP_LOGI(TAG, "recorder initialization starting");
    /* This firmware's tested capture mode. Reject unexpected geometry instead
     * of silently overflowing buffers or producing a malformed JPEG. */
    if (width != 1920 || height != 1080) return ESP_ERR_NOT_SUPPORTED;
    s_width = width; s_height = height;
    jpeg_encode_memory_alloc_cfg_t in = { .buffer_direction = JPEG_ENC_ALLOC_INPUT_BUFFER };
    jpeg_encode_memory_alloc_cfg_t out = { .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER };
    /* Reserve complete MCU rows, including padding beyond the last source row. */
    size_t bytes = (size_t)width * ((height + 15) & ~15u) * 2;
    s_raw = jpeg_alloc_encoder_mem(bytes, &in, &s_raw_size);
    s_jpeg = jpeg_alloc_encoder_mem(bytes, &out, &s_jpeg_size);
    s_index = heap_caps_malloc(INDEX_CAPACITY * sizeof(*s_index), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_file_buffer = heap_caps_malloc(FILE_BUFFER_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    /* Espressif's storage/perf_benchmark uses internal, cache-aligned DMA
     * buffers. Keep only a small staging block in scarce internal RAM. */
    s_dma_buffer = heap_caps_malloc(DMA_BUFFER_BYTES,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_CACHE_ALIGNED);
    esp_err_t ret = ESP_ERR_NO_MEM;
    if (!s_raw || !s_jpeg || !s_index || !s_file_buffer || !s_dma_buffer) goto fail;
    jpeg_encode_engine_cfg_t engine = { .timeout_ms = 1000 };
    ret = jpeg_new_encoder_engine(&engine, &s_encoder);
    if (ret != ESP_OK) goto fail;
    /* Put a present card into SPI mode before the first XPT2046 transfer.
     * A missing card is nonfatal; RECORD retries mounting it. */
    mount_card();
    s_jpeg_queue = xQueueCreate(JPEG_QUEUE_DEPTH, sizeof(jpeg_frame_t));
    if (!s_jpeg_queue) { ret = ESP_ERR_NO_MEM; goto fail; }
    if (xTaskCreate(writer_task, "sd_write", 6144, NULL, tskIDLE_PRIORITY + 1,
                    &s_writer_task) != pdPASS) {
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }
    TaskHandle_t task;
    if (xTaskCreate(recorder_task, "sd_record", 6144, NULL, tskIDLE_PRIORITY + 1, &task) != pdPASS) {
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }
    portENTER_CRITICAL(&s_lock);
    s_task = task;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "SD SPI3: SCLK=%d MOSI=%d MISO=%d CS=%d, requested max %d kHz",
             ICG_TOUCH_PIN_SCLK, ICG_TOUCH_PIN_MOSI, ICG_TOUCH_PIN_MISO,
             ICG_SD_PIN_CS, ICG_SD_CLOCK_KHZ);
    return ESP_OK;
fail:
    ESP_LOGE(TAG, "recorder initialization failed: %s; free internal=%u PSRAM=%u",
             esp_err_to_name(ret),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (s_writer_task) { vTaskDelete(s_writer_task); s_writer_task = NULL; }
    if (s_jpeg_queue) { vQueueDelete(s_jpeg_queue); s_jpeg_queue = NULL; }
    unmount_card();
    if (s_encoder) { jpeg_del_encoder_engine(s_encoder); s_encoder = NULL; }
    free(s_raw); free(s_jpeg); free(s_index); free(s_file_buffer); free(s_dma_buffer);
    s_raw = NULL; s_jpeg = NULL; s_index = NULL; s_file_buffer = NULL; s_dma_buffer = NULL;
    return ret;
}

void recorder_request(bool start)
{
    portENTER_CRITICAL(&s_lock);
    TaskHandle_t task = s_task;
    bool accepted = false;
    if (task) {
        if (start && s_state == REC_IDLE) {
            s_cancel_start = false;
            s_state = REC_STARTING;
            accepted = true;
        } else if (!start && s_state == REC_STARTING) {
            s_cancel_start = true;
            accepted = true;
        } else if (!start && s_state == REC_ACTIVE) {
            s_state = REC_STOPPING;
            accepted = true;
        }
    }
    portEXIT_CRITICAL(&s_lock);
    if (accepted) {
        ESP_LOGI(TAG, "%s request accepted", start ? "RECORD" : "STOP");
        xTaskNotifyGive(s_writer_task);
    } else if (task) {
        ESP_LOGI(TAG, "%s request ignored: recorder already starting/recording/stopping or idle",
                 start ? "RECORD" : "STOP");
    }
    if (start && !task) ESP_LOGW(TAG, "recorder not ready");
}

void recorder_submit(const uint8_t *rgb565, size_t len, size_t stride)
{
    size_t row_bytes = (size_t)s_width * 2;
    if (!stride) stride = row_bytes;
    if (!rgb565 || !row_bytes || stride < row_bytes || len < stride * (s_height - 1) + row_bytes) return;
    int64_t now = esp_timer_get_time();
    /* There is only one producer. A full-queue snapshot may conservatively
     * skip a frame just as the writer frees a slot, but never waits for SD.
     * Avoid spending ~70 ms copying and ~34 ms encoding a doomed frame. */
    bool queue_full = s_jpeg_queue && uxQueueSpacesAvailable(s_jpeg_queue) == 0;
    portENTER_CRITICAL(&s_lock);
    bool accept = s_state == REC_ACTIVE && now >= s_next_us && !s_busy;
    if (s_state == REC_ACTIVE && now >= s_next_us && s_busy) ++s_dropped;
    if (accept) {
        s_next_us = now + PERIOD_US;
        if (queue_full) {
            ++s_queue_drops;
            accept = false;
        } else {
            s_busy = true;
        }
    }
    portEXIT_CRITICAL(&s_lock);
    if (!accept) return;
    /* Preserve camera orientation and full sensor view. Skip source row
     * padding when present; tightly packed frames need only one copy. */
    if (stride == row_bytes) {
        memcpy(s_raw, rgb565, row_bytes * s_height);
    } else {
        for (uint32_t y = 0; y < s_height; ++y) {
            memcpy(s_raw + y * row_bytes, rgb565 + y * stride, row_bytes);
        }
    }
    for (uint32_t y = s_height; y < ((s_height + 15) & ~15u); ++y) {
        memcpy(s_raw + y * row_bytes, s_raw + (s_height - 1) * row_bytes, row_bytes);
    }
    portENTER_CRITICAL(&s_lock);
    s_copy_us = esp_timer_get_time() - now;
    s_frame_us = now;
    s_ready = true;
    portEXIT_CRITICAL(&s_lock);
    xTaskNotifyGive(s_task);
}

bool recorder_is_recording(void)
{
    portENTER_CRITICAL(&s_lock);
    bool active = s_state == REC_ACTIVE || s_state == REC_STOPPING;
    portEXIT_CRITICAL(&s_lock);
    return active;
}
