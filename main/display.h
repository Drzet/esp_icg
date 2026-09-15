#pragma once

#include <stdint.h>
#include "esp_err.h"

esp_err_t display_init(void);
esp_err_t display_draw_preview(const uint16_t *rgb565_frame);
