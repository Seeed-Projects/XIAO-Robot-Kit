#pragma once

#include <stdbool.h>

#include "esp_err.h"

/*
 * Two hobby servos on the carrier, driven by LEDC at 50 Hz / 14-bit:
 *   SERVO1 -> D15/GPIO42 -> J1  (RIGHT servo)
 *   SERVO2 -> D9 /GPIO8  -> J5  (LEFT  servo)
 * Calibrated Grove Servo pulse range 620..2500 us maps to 0..180 degrees.
 * Each arm retains the original 1500 us center at command angle 90, so installed
 * end effectors stay aligned without reassembly. A servo that is off outputs 0
 * duty (no holding torque).
 */
#define SERVO_COUNT 2
#define SERVO_ANGLE_MIN 0
#define SERVO_ANGLE_MAX 180

esp_err_t servo_init(void);
esp_err_t servo_set_angle(int servo_id, int angle);
esp_err_t servo_get_angle(int servo_id, int *angle);
esp_err_t servo_off(int servo_id);
void servo_print_status(void);
