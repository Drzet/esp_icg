#pragma once

#include "esp_err.h"

/* Stop the sensor through its existing esp_video/SCCB owner, then read back. */
esp_err_t p4d_camera_sensor_standby(int fd);

/* Read sensor state and sample receiver activity; never reconfigure hardware. */
void p4d_camera_diagnostics(int fd, const char *stage);
