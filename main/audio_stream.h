#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

esp_err_t audio_stream_start(i2c_master_bus_handle_t i2c_bus,
                             SemaphoreHandle_t serial_mutex);
