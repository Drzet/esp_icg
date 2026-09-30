#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct { uint32_t offset, size; } avi_index_t;
typedef struct {
    int fd;
    uint8_t *buffer, *io_buffer;
    size_t buffer_size, buffered, io_buffer_size;
    bool io_failed;
    avi_index_t *index;
    uint32_t capacity, frames, end, max_frame, width, height, period_us;
    int64_t first_us, last_us;
} avi_writer_t;

/* Caller owns fd and buffers. io_buffer must be DMA-capable on the target.
 * Buffer sizes must be multiples of the SD sector size (512 bytes).
 * Flush transfers bytes to VFS; only the caller synchronizes on STOP. */
bool avi_begin(avi_writer_t *a, int fd, avi_index_t *index, uint32_t capacity,
               uint32_t width, uint32_t height, uint32_t period_us,
               uint8_t *buffer, size_t buffer_size,
               uint8_t *io_buffer, size_t io_buffer_size);
bool avi_frame(avi_writer_t *a, const uint8_t *jpeg, uint32_t size, int64_t time_us);
bool avi_checkpoint(avi_writer_t *a);
bool avi_finish(avi_writer_t *a);
