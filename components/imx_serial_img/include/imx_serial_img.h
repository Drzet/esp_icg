/*
 * SPDX-FileCopyrightText: 2026 esp_cam_sensor_imx contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t imx_serial_send_blob(const char *name, const char *fmt,
                               uint32_t w, uint32_t h,
                               const uint8_t *data, size_t len,
                               const char *extra);

#ifdef __cplusplus
}
#endif
