#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Start a low-priority, read-only ISP diagnostics task.
 * The task only opens the ISP V4L2 device and issues VIDIOC_G_EXT_CTRLS.
 * It never writes camera or ISP controls.
 */
esp_err_t isp_diag_start(void);

#ifdef __cplusplus
}
#endif
