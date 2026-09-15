#pragma once

#include <stdbool.h>
#include "esp_err.h"

esp_err_t camera_init(void);
esp_err_t camera_start(void);
esp_err_t camera_stop(void);
bool camera_running(void);
