#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "imu_lsm6ds3tr.h"
#include "speed_control.h"

/* The 10-second IMU-guided spin action records at roughly 100 Hz. */
#define IMU_SPIN_LOG_MAX_SAMPLES 1200
#define IMU_SPIN_LOG_MAX_SESSION_ID 999

esp_err_t imu_spin_log_init(void);
esp_err_t imu_spin_log_start(int left_speed, int right_speed);
void imu_spin_log_append(int64_t timestamp_us, const imu_lsm6ds3tr_sample_t *sample,
                         float turn_rate_dps, float turned_degrees,
                         const speed_control_telemetry_t *speed_telemetry);
esp_err_t imu_spin_log_stop_and_save(size_t *sample_count, unsigned int *session_id);
esp_err_t imu_spin_log_list(void);
esp_err_t imu_spin_log_dump(unsigned int session_id);
esp_err_t imu_spin_log_delete(unsigned int session_id);
esp_err_t imu_spin_log_clear(void);
void imu_spin_log_print_status(void);
