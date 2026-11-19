#pragma once

#include "esp_err.h"

esp_err_t vofa_telemetry_init(void);
void vofa_telemetry_write(int set_left_speed, int set_right_speed,
                          int pid_left_target_cps, int pid_right_target_cps,
                          int left_encoder_count, int right_encoder_count,
                          int left_cps, int right_cps,
                          int left_pwm_percent, int right_pwm_percent);
