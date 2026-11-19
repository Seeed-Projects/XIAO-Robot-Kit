#pragma once

#include "esp_err.h"

/* User setpoint range. It is mapped to one common calibrated full-speed CPS. */
#define SPEED_CONTROL_SPEED_MAX 100

/* Atomic snapshot of the latest 50 ms PID update for diagnostic loggers. */
typedef struct {
    int left_target_cps;
    int right_target_cps;
    int left_measured_cps;
    int right_measured_cps;
    int left_pwm_percent;
    int right_pwm_percent;
} speed_control_telemetry_t;

esp_err_t speed_control_init(void);
esp_err_t speed_control_set_target(int left_speed, int right_speed);
/* Update a running closed-loop target without resetting wheel PID state. */
esp_err_t speed_control_update_target(int left_speed, int right_speed);
void speed_control_stop(void);
esp_err_t speed_control_get_telemetry(speed_control_telemetry_t *telemetry);
void speed_control_print_status(void);
