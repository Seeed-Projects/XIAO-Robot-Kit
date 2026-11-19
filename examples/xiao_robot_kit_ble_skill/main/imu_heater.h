#pragma once

#include "esp_err.h"

/* Starts the background PWM temperature controller for the IMU heater. */
esp_err_t imu_heater_init(void);
void imu_heater_print_status(void);
