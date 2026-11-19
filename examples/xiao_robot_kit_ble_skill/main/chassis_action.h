#pragma once

#include <stdbool.h>

#include "esp_err.h"

typedef enum {
    CHASSIS_ACTION_MOVE_FORWARD,
    CHASSIS_ACTION_MOVE_BACKWARD,
    CHASSIS_ACTION_GREETING,
    CHASSIS_ACTION_TURN_LEFT,
    CHASSIS_ACTION_TURN_RIGHT,
    CHASSIS_ACTION_REVERSE_PARK,
    CHASSIS_ACTION_SPIN_CLOCKWISE,
    CHASSIS_ACTION_SPIN_COUNTERCLOCKWISE,
    CHASSIS_ACTION_EMOTION_HAPPY,
    CHASSIS_ACTION_EMOTION_SCARED,
    CHASSIS_ACTION_EMOTION_ANGRY,
    CHASSIS_ACTION_COUNT,
} chassis_action_id_t;

esp_err_t chassis_action_init(void);
esp_err_t chassis_action_start(chassis_action_id_t action);
esp_err_t chassis_action_cancel(void);
bool chassis_action_is_active(void);
void chassis_action_print_status(void);

/* Record the fixed PCB installation attitude and gyro zero bias for spin actions. */
esp_err_t chassis_action_calibrate_imu(void);

/* Return the calibrated gravity-axis yaw rate used by IMU turn actions. */
esp_err_t chassis_action_read_calibrated_yaw_rate(float *rate_dps);

/* Onsite tuning; only saved reverse_park speeds persist across reboot. */
esp_err_t chassis_action_tune_set_speed(chassis_action_id_t action, int left_speed,
                                        int right_speed);
esp_err_t chassis_action_tune_save_reverse_park(void);
esp_err_t chassis_action_tune_reset(void);
void chassis_action_tune_print(void);
