#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t recorder_init(void);
void recorder_set_active(bool active);
bool recorder_active(void);
uint32_t recorder_dropped_frames(void);
void recorder_submit_rgb565(const uint8_t *buf, uint32_t width, uint32_t height, size_t len);
