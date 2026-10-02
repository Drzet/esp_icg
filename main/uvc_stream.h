#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define ICG_UVC_WIDTH 1920
#define ICG_UVC_HEIGHT 1080
#define ICG_UVC_QUALITY 80

esp_err_t uvc_stream_init(void);
bool uvc_stream_active(void);
/* Synchronous JPEG read of rgb565; caller can requeue raw buffer on return. */
void uvc_stream_submit(const uint8_t *rgb565, size_t len);
void uvc_stream_discard_ready(void);
void uvc_stream_disable(void);
