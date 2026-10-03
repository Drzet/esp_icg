#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#define P4D_CAMERA_WIDTH 1280
#define P4D_CAMERA_HEIGHT 720

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*p4d_camera_frame_cb_t)(
    const uint8_t *data,
    size_t len,
    uint32_t width,
    uint32_t height,
    void *user_ctx);

/**
 * Initialize the WT9932P4-TINY MIPI-CSI path through esp_video using the
 * caller's shared I2C bus. The board exposes no camera reset or PWDN GPIO.
 */
esp_err_t p4d_camera_init(i2c_master_bus_handle_t i2c_bus);

/** Start OV9281 1280x720 RAW8 capture using the Wireless-Tag BSP CSI flow. */
esp_err_t p4d_camera_start(p4d_camera_frame_cb_t cb, void *user_ctx);

#ifdef __cplusplus
}
#endif
