#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

void ble_mouse_start(SemaphoreHandle_t serial_mutex);
void ble_mouse_update_gyro(float gyro_x_dps, float gyro_z_dps);
