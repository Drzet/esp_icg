#pragma once

#include <stdbool.h>
#include "esp_err.h"

esp_err_t storage_init(void);
bool storage_ready(void);
const char *storage_mount_point(void);
