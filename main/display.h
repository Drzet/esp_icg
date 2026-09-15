#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t display_init(void);
esp_err_t display_draw_preview(const uint16_t *rgb565);
esp_err_t display_draw_ui(bool camera_running, bool recording, bool sd_ready);
