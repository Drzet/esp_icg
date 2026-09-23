#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t recorder_init(uint32_t width, uint32_t height);
/* Thread-safe, nonblocking commands. Start while active is a no-op. */
void recorder_request(bool start);
/* Called before requeueing a camera buffer; accepted frames remain owned until JPEG finishes. */
void recorder_submit(const uint8_t *rgb565, size_t len, size_t stride);

/* True while accepting frames or draining/finalizing a recording. */
bool recorder_is_recording(void);
