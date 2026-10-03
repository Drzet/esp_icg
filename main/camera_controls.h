#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Standard UVC entity IDs and selectors (UVC 1.5, chapter 4). */
enum { ICG_CT = 1, ICG_PU = 3 };
enum { CT_AE_MODE = 2, CT_EXPOSURE = 4, CT_FOCUS = 6 };
enum { PU_BRIGHTNESS = 2, PU_CONTRAST = 3, PU_GAIN = 4, PU_HUE = 6,
       PU_SATURATION = 7, PU_SHARPNESS = 8, PU_GAMMA = 9,
       PU_WB_COMPONENT = 12, PU_WB_AUTO = 13 };
enum { CTRL_OK = 0, CTRL_NOT_READY = 1, CTRL_WRONG_STATE = 2,
       CTRL_RANGE = 4, CTRL_INVALID_CONTROL = 6, CTRL_INVALID_REQUEST = 7 };
typedef struct {
    uint8_t entity, selector, bit, size;
    bool signed_value, enabled;
    int32_t min, max, res, def;
    uint32_t id, ipa_mask;
} icg_control_t;

esp_err_t camera_controls_init(int camera_fd);
const icg_control_t *camera_control_find(uint8_t entity, uint8_t selector);
uint32_t camera_control_bitmap(uint8_t entity);
uint8_t camera_control_info(const icg_control_t *c);
int camera_control_get(const icg_control_t *c, uint32_t *value);
int camera_control_set(const icg_control_t *c, uint32_t value);
