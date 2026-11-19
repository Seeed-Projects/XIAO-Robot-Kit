#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * Raw LSM6DS3TR-C sample.  The configured full scales are reported by
 * imu_print_status(); raw values are intentionally exposed for bring-up and
 * axis-orientation verification before any chassis-frame transform is added.
 */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int32_t temperature_mC;
} imu_lsm6ds3tr_sample_t;

esp_err_t imu_init(void);
esp_err_t imu_reinit(void);
bool imu_is_ready(void);
esp_err_t imu_read_sample(imu_lsm6ds3tr_sample_t *sample);
esp_err_t imu_read_register(uint8_t reg, uint8_t *value);
void imu_print_status(void);
