#pragma once

#include <stdbool.h>

#include "esp_err.h"

/*
 * XIAO Robot Board V1.0: two GC9112 H-bridge drivers (U14, U16), each running
 * one gear motor. The chassis is a two-wheel differential drive.
 *
 *   M1 -> U14, IN1=D1 /GPIO2,  IN2=D2 /GPIO3,  OUT1/OUT2 -> J4 / J8  (RIGHT motor, J8)
 *   M2 -> U16, IN1=D19/GPIO11, IN2=D18/GPIO12, OUT1/OUT2 -> J6 / J9  (LEFT  motor, J9)
 *
 * Assembly frozen on V1.0: M1 (J8) drives the RIGHT wheel and M2 (J9) the LEFT.
 * Only the electrical polarity of each motor is left to calibrate on the bench,
 * via the runtime invert flag (motor_control_set_inverted).
 */
#define MOTOR_COUNT 2
#define MOTOR_SPEED_MAX 100

/* Differential mapping: index 0 (M1) = right wheel, index 1 (M2) = left wheel. */
#define MOTOR_LEFT_INDEX 1
#define MOTOR_RIGHT_INDEX 0

esp_err_t motor_control_init(void);
esp_err_t motor_control_set(int motor_id, int speed_percent);
esp_err_t motor_control_drive(int left_speed_percent, int right_speed_percent);
esp_err_t motor_control_get_speed(int motor_id, int *speed_percent);
void motor_control_stop_all(void);

/* Bench calibration: flip a motor's electrical polarity at runtime. */
esp_err_t motor_control_set_inverted(int motor_id, bool inverted);
bool motor_control_is_inverted(int motor_id);

void motor_control_print_status(void);
