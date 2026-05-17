#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

esp_err_t camera_app_init(void);
bool camera_app_is_ready(void);
esp_err_t camera_app_get_latest_jpeg(uint8_t **out_buf, size_t *out_len, TickType_t wait_ticks);
void camera_app_free_jpeg(uint8_t *buf);
