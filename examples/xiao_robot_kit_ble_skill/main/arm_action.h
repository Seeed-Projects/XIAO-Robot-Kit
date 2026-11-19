#pragma once

#include <stdbool.h>

#include "esp_err.h"

typedef enum {
    ARM_ACTION_HANDSHAKE,
    ARM_ACTION_WAVE_HAND,
    ARM_ACTION_OPEN_ARMS,
    ARM_ACTION_RAISE_BOTH_ARMS,
    ARM_ACTION_CLOSE_ARMS,
    ARM_ACTION_EMOTION_HAPPY,
    /* Internal preset: keeps waving until the chassis ends its IMU-guided spin. */
    ARM_ACTION_EMOTION_HAPPY_SPIN,
    /* Internal preset: one direct closed-pose command, then hold it while settling. */
    ARM_ACTION_EMOTION_HAPPY_HOME,
    /* Timed to the 6-second local "请注意倒车" BGM. */
    ARM_ACTION_REVERSE_PARK,
    ARM_ACTION_EMOTION_SCARED,
    ARM_ACTION_EMOTION_ANGRY,
} arm_action_id_t;

/* Runtime-tunable key poses used by the built-in arm actions. */
typedef enum {
    ARM_TUNE_POSE_HOME,
    ARM_TUNE_POSE_OPEN,
    ARM_TUNE_POSE_OPEN_SWING,
    ARM_TUNE_POSE_RAISED,
    ARM_TUNE_POSE_HANDSHAKE_READY,
    ARM_TUNE_POSE_HANDSHAKE_HIGH,
    ARM_TUNE_POSE_HANDSHAKE_LOW,
    ARM_TUNE_POSE_WAVE_READY,
    ARM_TUNE_POSE_WAVE_HIGH,
    ARM_TUNE_POSE_WAVE_LOW,
    ARM_TUNE_POSE_HAPPY_A,
    ARM_TUNE_POSE_HAPPY_B,
    ARM_TUNE_POSE_REVERSE_PARK_A,
    ARM_TUNE_POSE_REVERSE_PARK_B,
    ARM_TUNE_POSE_COUNT,
} arm_tune_pose_t;

esp_err_t arm_action_init(void);
esp_err_t arm_action_start(arm_action_id_t action);
esp_err_t arm_action_cancel(void);
bool arm_action_is_active(void);
void arm_action_print_status(void);

/* Onsite tuning; changes are RAM-only and may only be applied while idle. */
esp_err_t arm_action_tune_set_pose(arm_tune_pose_t pose, int right_angle, int left_angle);
esp_err_t arm_action_tune_reset(void);
void arm_action_tune_print(void);
