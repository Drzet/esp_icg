#pragma once
#include <stdbool.h>
#include "esp_err.h"
esp_err_t shared_spi_init(void);
bool shared_spi_touch_lock(void);
void shared_spi_mount_lock(void);
void shared_spi_unlock(void);
