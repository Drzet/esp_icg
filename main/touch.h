#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    int x;
    int y;
    uint16_t raw_x;
    uint16_t raw_y;
} touch_point_t;

esp_err_t touch_init(void);
bool touch_read(touch_point_t *point);

bool touch_is_pressed(void);
