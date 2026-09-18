#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef struct { uint32_t offset, size; } avi_index_t;
typedef struct {
    FILE *file;
    avi_index_t *index;
    uint32_t capacity, frames, end, max_frame, width, height, period_us;
    int64_t first_us, last_us;
} avi_writer_t;

bool avi_begin(avi_writer_t *a, FILE *file, avi_index_t *index, uint32_t capacity,
               uint32_t width, uint32_t height, uint32_t period_us);
bool avi_frame(avi_writer_t *a, const uint8_t *jpeg, uint32_t size, int64_t time_us);
bool avi_checkpoint(avi_writer_t *a);
bool avi_finish(avi_writer_t *a);
