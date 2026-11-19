#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/* 25 ms (40 Hz) samples for 30 seconds, held in a fixed-size RAM buffer. */
#define CPS_LOG_MAX_SAMPLES 1200
#define CPS_LOG_SAMPLE_PERIOD_MS 25
#define CPS_LOG_MAX_SESSION_ID 999

esp_err_t cps_log_init(void);
esp_err_t cps_log_start(void);
esp_err_t cps_log_stop_and_save(size_t *sample_count, unsigned int *session_id);
esp_err_t cps_log_list(void);
esp_err_t cps_log_summary(void);
esp_err_t cps_log_clear(void);
esp_err_t cps_log_delete(unsigned int session_id);
esp_err_t cps_log_dump(unsigned int session_id);
void cps_log_print_status(void);

void cps_log_append(int64_t timestamp_us, int set_left_speed, int set_right_speed,
                    int target_left_cps, int target_right_cps,
                    int measured_left_cps, int measured_right_cps,
                    int left_pwm_percent, int right_pwm_percent);
