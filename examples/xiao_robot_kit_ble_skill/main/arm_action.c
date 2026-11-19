#include "arm_action.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "servo.h"

/* Servo numbering is fixed by the carrier schematic and the shell interface. */
#define RIGHT_ARM_SERVO 1
#define LEFT_ARM_SERVO 2
#define ARM_ANGLE_KEEP (-1)

/* Hold each target for two 50 Hz PWM periods before issuing the next change. */
#define ARM_MOTION_FRAME_MS 40
/*
 * One-degree target updates fall inside the gear/potentiometer deadband of the
 * installed servos.  Filtering them prevents the control loop from hunting,
 * particularly when gravity assists a downward arm movement.
 */
#define ARM_MIN_COMMAND_DELTA_DEG 3
#define ARM_HANDSHAKE_APPROACH_MS 250
#define ARM_HANDSHAKE_SWING_MS 160
#define ARM_HANDSHAKE_HOME_MS 300
#define HANDSHAKE_PAUSE_MS 300
#define HANDSHAKE_SWING_COUNT 3
#define WAVE_SWING_COUNT 4
#define ARM_WAVE_APPROACH_MS 400
#define ARM_WAVE_SWING_MS 160
#define ARM_WAVE_HOME_MS 300
#define ARM_OPEN_SWING_COUNT 2
#define ARM_OPEN_SWING_MS 200
#define ARM_POSE_TRANSITION_MS 700
#define ARM_EMOTION_HAPPY_RAISE_MS 250
#define ARM_EMOTION_HAPPY_SWING_COUNT 2
#define ARM_EMOTION_HAPPY_SWING_MS 250
#define ARM_EMOTION_SCARED_CLOSE_MS 250
#define ARM_EMOTION_ANGRY_OPEN_MS 250
#define ARM_EMOTION_ANGRY_CLOSE_MS 200
#define ARM_EMOTION_ANGRY_CLOSE_COUNT 2

#define ARM_ACTION_TASK_STACK_SIZE 2048
#define ARM_ACTION_TASK_PRIORITY 4
#define ARM_ACTION_CANCEL_TIMEOUT_MS 500

static const char *TAG = "arm_action";

static SemaphoreHandle_t s_mutex;
static SemaphoreHandle_t s_completion;
static TaskHandle_t s_task;
static bool s_cancel_requested;
static arm_action_id_t s_current_action;

typedef struct {
    int right_angle;
    int left_angle;
} arm_pose_t;

typedef struct {
    arm_tune_pose_t pose;
    uint32_t duration_ms;
} timed_arm_pose_t;

/* Defaults are copied to s_tuned_poses at boot and never persisted. */
static const arm_pose_t s_default_poses[ARM_TUNE_POSE_COUNT] = {
    [ARM_TUNE_POSE_HOME] = { 0, 180 },
    [ARM_TUNE_POSE_OPEN] = { 180, 0 },
    [ARM_TUNE_POSE_OPEN_SWING] = { 160, 20 },
    [ARM_TUNE_POSE_RAISED] = { 130, 50 },
    [ARM_TUNE_POSE_HANDSHAKE_READY] = { 40, 180 },
    [ARM_TUNE_POSE_HANDSHAKE_HIGH] = { 52, 180 },
    [ARM_TUNE_POSE_HANDSHAKE_LOW] = { 28, 180 },
    [ARM_TUNE_POSE_WAVE_READY] = { 135, 180 },
    [ARM_TUNE_POSE_WAVE_HIGH] = { 155, 180 },
    [ARM_TUNE_POSE_WAVE_LOW] = { 115, 180 },
    [ARM_TUNE_POSE_HAPPY_A] = { 170, 90 },
    [ARM_TUNE_POSE_HAPPY_B] = { 90, 10 },
    /* "请注意倒车8s": audible 0.05-8.38 s, 136 BPM, 441.2 ms per beat. */
    [ARM_TUNE_POSE_REVERSE_PARK_A] = { 65, 25 },
    [ARM_TUNE_POSE_REVERSE_PARK_B] = { 155, 115 },
};

static arm_pose_t s_tuned_poses[ARM_TUNE_POSE_COUNT] = {
    [ARM_TUNE_POSE_HOME] = { 0, 180 },
    [ARM_TUNE_POSE_OPEN] = { 180, 0 },
    [ARM_TUNE_POSE_OPEN_SWING] = { 160, 20 },
    [ARM_TUNE_POSE_RAISED] = { 130, 50 },
    [ARM_TUNE_POSE_HANDSHAKE_READY] = { 40, 180 },
    [ARM_TUNE_POSE_HANDSHAKE_HIGH] = { 52, 180 },
    [ARM_TUNE_POSE_HANDSHAKE_LOW] = { 28, 180 },
    [ARM_TUNE_POSE_WAVE_READY] = { 135, 180 },
    [ARM_TUNE_POSE_WAVE_HIGH] = { 155, 180 },
    [ARM_TUNE_POSE_WAVE_LOW] = { 115, 180 },
    [ARM_TUNE_POSE_HAPPY_A] = { 170, 90 },
    [ARM_TUNE_POSE_HAPPY_B] = { 90, 10 },
    [ARM_TUNE_POSE_REVERSE_PARK_A] = { 65, 25 },
    [ARM_TUNE_POSE_REVERSE_PARK_B] = { 155, 115 },
};

/*
 * "请注意倒车8s" arm guide: poses alternate on every measured beat of the
 * BGM (441 ms grid), switch to eighth notes for the 6.1-7.4 s drum fill, and
 * start gliding home on the last accent so the arms park as the music fades.
 * The lead covers the phone player's start latency; keep it in sync with
 * CHASSIS_ACTION_REVERSE_PARK_LEAD_MS and calibrate in 100 ms steps.
 */
#define ARM_REVERSE_PARK_LEAD_MS 600
static const timed_arm_pose_t reverse_park_arm_beats[] = {
    { ARM_TUNE_POSE_HOME, ARM_REVERSE_PARK_LEAD_MS },
    /* beat 1 @0.09s */ { ARM_TUNE_POSE_REVERSE_PARK_A, 441 },
    /* beat 2 @0.53s */ { ARM_TUNE_POSE_REVERSE_PARK_B, 441 },
    /* beat 3 @0.97s */ { ARM_TUNE_POSE_REVERSE_PARK_A, 418 },
    /* beat 4 @1.39s */ { ARM_TUNE_POSE_REVERSE_PARK_B, 441 },
    /* beat 5 @1.83s */ { ARM_TUNE_POSE_REVERSE_PARK_A, 418 },
    /* beat 6 @2.25s */ { ARM_TUNE_POSE_REVERSE_PARK_B, 442 },
    /* beat 7 @2.69s */ { ARM_TUNE_POSE_REVERSE_PARK_A, 417 },
    /* beat 8 @3.11s */ { ARM_TUNE_POSE_REVERSE_PARK_B, 418 },
    /* beat 9 @3.53s */ { ARM_TUNE_POSE_REVERSE_PARK_A, 442 },
    /* beat 10 @3.97s */ { ARM_TUNE_POSE_REVERSE_PARK_B, 418 },
    /* beat 11 @4.39s */ { ARM_TUNE_POSE_REVERSE_PARK_A, 441 },
    /* beat 12 @4.83s */ { ARM_TUNE_POSE_REVERSE_PARK_B, 418 },
    /* beat 13 @5.25s */ { ARM_TUNE_POSE_REVERSE_PARK_A, 441 },
    /* beat 14 @5.69s */ { ARM_TUNE_POSE_REVERSE_PARK_B, 418 },
    /* fill @6.11s */ { ARM_TUNE_POSE_REVERSE_PARK_A, 221 },
    /* fill @6.33s */ { ARM_TUNE_POSE_REVERSE_PARK_B, 220 },
    /* fill @6.55s */ { ARM_TUNE_POSE_REVERSE_PARK_A, 209 },
    /* fill @6.76s */ { ARM_TUNE_POSE_REVERSE_PARK_B, 209 },
    /* fill @6.97s */ { ARM_TUNE_POSE_REVERSE_PARK_A, 221 },
    /* fill @7.19s */ { ARM_TUNE_POSE_REVERSE_PARK_B, 220 },
    /* beat 18 @7.41s */ { ARM_TUNE_POSE_REVERSE_PARK_A, 418 },
    /* beat 19 @7.83s */ { ARM_TUNE_POSE_REVERSE_PARK_B, 441 },
    /* music ends @8.38s: glide home */ { ARM_TUNE_POSE_HOME, 600 },
};

static const char *action_name(arm_action_id_t action)
{
    switch (action) {
    case ARM_ACTION_HANDSHAKE:
        return "handshake";
    case ARM_ACTION_WAVE_HAND:
        return "wave_hand";
    case ARM_ACTION_OPEN_ARMS:
        return "open_arms";
    case ARM_ACTION_RAISE_BOTH_ARMS:
        return "raise_both_arms";
    case ARM_ACTION_CLOSE_ARMS:
        return "close_arms";
    case ARM_ACTION_EMOTION_HAPPY:
        return "emotion_happy";
    case ARM_ACTION_EMOTION_HAPPY_SPIN:
        return "emotion_happy_spin";
    case ARM_ACTION_EMOTION_HAPPY_HOME:
        return "emotion_happy_home";
    case ARM_ACTION_REVERSE_PARK:
        return "reverse_park_arms";
    case ARM_ACTION_EMOTION_SCARED:
        return "emotion_scared";
    case ARM_ACTION_EMOTION_ANGRY:
        return "emotion_angry";
    default:
        return "unknown";
    }
}

static const char *tune_pose_name(arm_tune_pose_t pose)
{
    switch (pose) {
    case ARM_TUNE_POSE_HOME: return "home";
    case ARM_TUNE_POSE_OPEN: return "open";
    case ARM_TUNE_POSE_OPEN_SWING: return "open_swing";
    case ARM_TUNE_POSE_RAISED: return "raised";
    case ARM_TUNE_POSE_HANDSHAKE_READY: return "handshake_ready";
    case ARM_TUNE_POSE_HANDSHAKE_HIGH: return "handshake_high";
    case ARM_TUNE_POSE_HANDSHAKE_LOW: return "handshake_low";
    case ARM_TUNE_POSE_WAVE_READY: return "wave_ready";
    case ARM_TUNE_POSE_WAVE_HIGH: return "wave_high";
    case ARM_TUNE_POSE_WAVE_LOW: return "wave_low";
    case ARM_TUNE_POSE_HAPPY_A: return "happy_a";
    case ARM_TUNE_POSE_HAPPY_B: return "happy_b";
    case ARM_TUNE_POSE_REVERSE_PARK_A: return "park_a";
    case ARM_TUNE_POSE_REVERSE_PARK_B: return "park_b";
    default: return "unknown";
    }
}

static bool tune_pose_is_valid(arm_tune_pose_t pose)
{
    return pose >= ARM_TUNE_POSE_HOME && pose < ARM_TUNE_POSE_COUNT;
}

static arm_pose_t tuned_pose(arm_tune_pose_t pose)
{
    return s_tuned_poses[pose];
}

static bool cancellation_requested(void)
{
    bool cancelled;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    cancelled = s_cancel_requested;
    xSemaphoreGive(s_mutex);
    return cancelled;
}

static bool set_arm_angles(int right_angle, int left_angle)
{
    if (cancellation_requested()) {
        return false;
    }

    /*
     * The two LEDC writes are consecutive, so a two-arm pose changes as close
     * to simultaneously as the peripheral interface permits.
     */
    if (right_angle != ARM_ANGLE_KEEP &&
        servo_set_angle(RIGHT_ARM_SERVO, right_angle) != ESP_OK) {
        ESP_LOGE(TAG, "set right arm to %d degrees failed", right_angle);
        return false;
    }
    if (left_angle != ARM_ANGLE_KEEP && servo_set_angle(LEFT_ARM_SERVO, left_angle) != ESP_OK) {
        ESP_LOGE(TAG, "set left arm to %d degrees failed", left_angle);
        return false;
    }
    return !cancellation_requested();
}

static bool wait_or_cancel(uint32_t duration_ms)
{
    return ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(duration_ms)) == 0 &&
           !cancellation_requested();
}

/* Small swings look cleaner when the servo can settle at each distinct pose. */
static bool hold_arm_angles(int right_angle, int left_angle, uint32_t duration_ms)
{
    return set_arm_angles(right_angle, left_angle) && wait_or_cancel(duration_ms);
}

static bool hold_tuned_pose(arm_tune_pose_t pose, uint32_t duration_ms)
{
    arm_pose_t angles = tuned_pose(pose);
    return hold_arm_angles(angles.right_angle, angles.left_angle, duration_ms);
}

static bool get_start_angle(int servo_id, int target_angle, int *start_angle)
{
    esp_err_t err = servo_get_angle(servo_id, start_angle);
    if (err == ESP_OK) {
        return true;
    }

    /* With PWM disabled, the physical position is unknown; safely re-enable at the target. */
    if (err == ESP_ERR_INVALID_STATE && servo_set_angle(servo_id, target_angle) == ESP_OK) {
        *start_angle = target_angle;
        return true;
    }

    ESP_LOGE(TAG, "read servo %d start angle failed: %s", servo_id, esp_err_to_name(err));
    return false;
}

static int eased_angle(int start_angle, int target_angle, uint32_t frame, uint32_t frame_count)
{
    /* Smoothstep: zero velocity at both ends prevents an abrupt visual reversal. */
    int64_t progress = ((int64_t)frame * 1000) / frame_count;
    int64_t eased = (progress * progress * (3000 - 2 * progress)) / 1000000;
    return start_angle + (int)(((int64_t)(target_angle - start_angle) * eased) / 1000);
}

static bool command_angle_changed_enough(int last_command, int requested_angle, bool is_final)
{
    int difference = requested_angle - last_command;
    if (difference < 0) {
        difference = -difference;
    }
    return is_final || difference >= ARM_MIN_COMMAND_DELTA_DEG;
}

static bool move_arm_angles(int right_target, int left_target, uint32_t duration_ms)
{
    int right_start = 0;
    int left_start = 0;
    if ((right_target != ARM_ANGLE_KEEP &&
         !get_start_angle(RIGHT_ARM_SERVO, right_target, &right_start)) ||
        (left_target != ARM_ANGLE_KEEP &&
         !get_start_angle(LEFT_ARM_SERVO, left_target, &left_start))) {
        return false;
    }

    uint32_t frame_count = (duration_ms + ARM_MOTION_FRAME_MS - 1) / ARM_MOTION_FRAME_MS;
    if (frame_count == 0) {
        frame_count = 1;
    }

    int last_right_command = right_start;
    int last_left_command = left_start;
    for (uint32_t frame = 1; frame <= frame_count; ++frame) {
        bool is_final_frame = frame == frame_count;
        int right_angle = ARM_ANGLE_KEEP;
        int left_angle = ARM_ANGLE_KEEP;
        if (right_target != ARM_ANGLE_KEEP) {
            int requested = eased_angle(right_start, right_target, frame, frame_count);
            if (command_angle_changed_enough(last_right_command, requested, is_final_frame)) {
                right_angle = requested;
                last_right_command = requested;
            }
        }
        if (left_target != ARM_ANGLE_KEEP) {
            int requested = eased_angle(left_start, left_target, frame, frame_count);
            if (command_angle_changed_enough(last_left_command, requested, is_final_frame)) {
                left_angle = requested;
                last_left_command = requested;
            }
        }
        if (!set_arm_angles(right_angle, left_angle)) {
            return false;
        }
        if (frame < frame_count && !wait_or_cancel(ARM_MOTION_FRAME_MS)) {
            return false;
        }
    }
    return true;
}

static bool move_tuned_pose(arm_tune_pose_t pose, uint32_t duration_ms)
{
    arm_pose_t angles = tuned_pose(pose);
    return move_arm_angles(angles.right_angle, angles.left_angle, duration_ms);
}

static bool run_handshake(void)
{
    if (!move_tuned_pose(ARM_TUNE_POSE_HANDSHAKE_READY, ARM_HANDSHAKE_APPROACH_MS)) {
        return false;
    }

    for (int swing = 0; swing < HANDSHAKE_SWING_COUNT; ++swing) {
        if (!hold_tuned_pose(ARM_TUNE_POSE_HANDSHAKE_HIGH, ARM_HANDSHAKE_SWING_MS) ||
            !hold_tuned_pose(ARM_TUNE_POSE_HANDSHAKE_LOW, ARM_HANDSHAKE_SWING_MS)) {
            return false;
        }
    }

    return hold_tuned_pose(ARM_TUNE_POSE_HANDSHAKE_READY, ARM_HANDSHAKE_SWING_MS) &&
           wait_or_cancel(HANDSHAKE_PAUSE_MS) &&
           hold_tuned_pose(ARM_TUNE_POSE_HOME, ARM_HANDSHAKE_HOME_MS);
}

static bool run_wave_hand(void)
{
    if (!move_tuned_pose(ARM_TUNE_POSE_WAVE_READY, ARM_WAVE_APPROACH_MS)) {
        return false;
    }

    for (int swing = 0; swing < WAVE_SWING_COUNT; ++swing) {
        if (!hold_tuned_pose(ARM_TUNE_POSE_WAVE_HIGH, ARM_WAVE_SWING_MS) ||
            !hold_tuned_pose(ARM_TUNE_POSE_WAVE_LOW, ARM_WAVE_SWING_MS)) {
            return false;
        }
    }

    return hold_tuned_pose(ARM_TUNE_POSE_HOME, ARM_WAVE_HOME_MS);
}

static bool run_open_arms(void)
{
    if (!move_tuned_pose(ARM_TUNE_POSE_OPEN, ARM_POSE_TRANSITION_MS)) {
        return false;
    }

    for (int swing = 0; swing < ARM_OPEN_SWING_COUNT; ++swing) {
        if (!hold_tuned_pose(ARM_TUNE_POSE_OPEN_SWING, ARM_OPEN_SWING_MS) ||
            !hold_tuned_pose(ARM_TUNE_POSE_OPEN, ARM_OPEN_SWING_MS)) {
            return false;
        }
    }
    return true;
}

static bool run_raise_both_arms(void)
{
    return move_tuned_pose(ARM_TUNE_POSE_RAISED, ARM_POSE_TRANSITION_MS);
}

static bool run_close_arms(void)
{
    return hold_tuned_pose(ARM_TUNE_POSE_HOME, ARM_POSE_TRANSITION_MS);
}

static bool run_emotion_happy_home(void)
{
    return hold_tuned_pose(ARM_TUNE_POSE_HOME, ARM_POSE_TRANSITION_MS);
}

static bool run_emotion_happy(void)
{
    if (!move_tuned_pose(ARM_TUNE_POSE_RAISED, ARM_EMOTION_HAPPY_RAISE_MS)) {
        return false;
    }

    for (int swing = 0; swing < ARM_EMOTION_HAPPY_SWING_COUNT; ++swing) {
        if (!hold_tuned_pose(ARM_TUNE_POSE_HAPPY_A, ARM_EMOTION_HAPPY_SWING_MS) ||
            !hold_tuned_pose(ARM_TUNE_POSE_HAPPY_B, ARM_EMOTION_HAPPY_SWING_MS)) {
            return false;
        }
    }
    return run_emotion_happy_home();
}

/*
 * The chassis owns the end time of a 360-degree spin, which varies with the
 * floor and battery. Keep the arms waving until that task cancels us so the
 * visual motion covers the entire rotation instead of relying on a fixed
 * number of swings.
 */
static bool run_emotion_happy_spin(void)
{
    if (!move_tuned_pose(ARM_TUNE_POSE_RAISED, ARM_EMOTION_HAPPY_RAISE_MS)) {
        return false;
    }

    while (!cancellation_requested()) {
        if (!hold_tuned_pose(ARM_TUNE_POSE_HAPPY_A, ARM_EMOTION_HAPPY_SWING_MS) ||
            !hold_tuned_pose(ARM_TUNE_POSE_HAPPY_B, ARM_EMOTION_HAPPY_SWING_MS)) {
            return false;
        }
    }
    return false;
}

static bool run_reverse_park(void)
{
    for (size_t step = 0;
         step < sizeof(reverse_park_arm_beats) / sizeof(reverse_park_arm_beats[0]); ++step) {
        const timed_arm_pose_t *pose = &reverse_park_arm_beats[step];
        if (!hold_tuned_pose(pose->pose, pose->duration_ms)) {
            return false;
        }
    }
    return true;
}

static bool run_emotion_scared(void)
{
    return hold_tuned_pose(ARM_TUNE_POSE_HOME, ARM_EMOTION_SCARED_CLOSE_MS);
}

static bool run_emotion_angry(void)
{
    if (!hold_tuned_pose(ARM_TUNE_POSE_OPEN, ARM_EMOTION_ANGRY_OPEN_MS)) {
        return false;
    }

    for (int close = 0; close < ARM_EMOTION_ANGRY_CLOSE_COUNT; ++close) {
        if (!hold_tuned_pose(ARM_TUNE_POSE_HOME, ARM_EMOTION_ANGRY_CLOSE_MS)) {
            return false;
        }
        if (close + 1 < ARM_EMOTION_ANGRY_CLOSE_COUNT &&
            !hold_tuned_pose(ARM_TUNE_POSE_OPEN, ARM_EMOTION_ANGRY_OPEN_MS)) {
            return false;
        }
    }
    return true;
}

static void arm_action_task(void *arg)
{
    arm_action_id_t action = (arm_action_id_t)(uintptr_t)arg;
    bool completed;

    switch (action) {
    case ARM_ACTION_HANDSHAKE:
        completed = run_handshake();
        break;
    case ARM_ACTION_WAVE_HAND:
        completed = run_wave_hand();
        break;
    case ARM_ACTION_OPEN_ARMS:
        completed = run_open_arms();
        break;
    case ARM_ACTION_RAISE_BOTH_ARMS:
        completed = run_raise_both_arms();
        break;
    case ARM_ACTION_CLOSE_ARMS:
        completed = run_close_arms();
        break;
    case ARM_ACTION_EMOTION_HAPPY:
        completed = run_emotion_happy();
        break;
    case ARM_ACTION_EMOTION_HAPPY_SPIN:
        completed = run_emotion_happy_spin();
        break;
    case ARM_ACTION_EMOTION_HAPPY_HOME:
        completed = run_emotion_happy_home();
        break;
    case ARM_ACTION_REVERSE_PARK:
        completed = run_reverse_park();
        break;
    case ARM_ACTION_EMOTION_SCARED:
        completed = run_emotion_scared();
        break;
    case ARM_ACTION_EMOTION_ANGRY:
        completed = run_emotion_angry();
        break;
    default:
        completed = false;
        break;
    }

    bool cancelled = cancellation_requested();
    if (completed) {
        ESP_LOGI(TAG, "%s complete", action_name(action));
    } else if (cancelled) {
        ESP_LOGI(TAG, "%s cancelled; arm positions kept", action_name(action));
    } else {
        ESP_LOGE(TAG, "%s aborted after servo write error", action_name(action));
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_task = NULL;
    s_cancel_requested = false;
    xSemaphoreGive(s_mutex);
    xSemaphoreGive(s_completion);
    vTaskDelete(NULL);
}

esp_err_t arm_action_init(void)
{
    if (s_mutex != NULL) {
        return ESP_OK;
    }

    s_mutex = xSemaphoreCreateMutex();
    s_completion = xSemaphoreCreateBinary();
    if (s_mutex == NULL || s_completion == NULL) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t arm_action_start(arm_action_id_t action)
{
    if (s_mutex == NULL || s_completion == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_task != NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    /* A completed task leaves one token; remove it before starting a new action. */
    while (xSemaphoreTake(s_completion, 0) == pdTRUE) {
    }

    s_current_action = action;
    s_cancel_requested = false;
    if (xTaskCreate(arm_action_task, action_name(action), ARM_ACTION_TASK_STACK_SIZE,
                    (void *)(uintptr_t)action, ARM_ACTION_TASK_PRIORITY, &s_task) != pdPASS) {
        s_task = NULL;
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t arm_action_cancel(void)
{
    if (s_mutex == NULL || s_completion == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    TaskHandle_t task = s_task;
    if (task == NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_OK;
    }
    s_cancel_requested = true;
    xTaskNotifyGive(task);
    xSemaphoreGive(s_mutex);

    return xSemaphoreTake(s_completion, pdMS_TO_TICKS(ARM_ACTION_CANCEL_TIMEOUT_MS)) == pdTRUE
               ? ESP_OK
               : ESP_ERR_TIMEOUT;
}

bool arm_action_is_active(void)
{
    if (s_mutex == NULL) {
        return false;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool active = s_task != NULL;
    xSemaphoreGive(s_mutex);
    return active;
}

void arm_action_print_status(void)
{
    if (s_mutex == NULL) {
        printf("Arm action: unavailable\n");
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_task == NULL) {
        printf("Arm action: idle\n");
    } else {
        printf("Arm action: %s%s\n", action_name(s_current_action),
               s_cancel_requested ? " (cancelling)" : "");
    }
    xSemaphoreGive(s_mutex);
}

esp_err_t arm_action_tune_set_pose(arm_tune_pose_t pose, int right_angle, int left_angle)
{
    if (s_mutex == NULL || !tune_pose_is_valid(pose) || right_angle < 0 ||
        right_angle > 180 || left_angle < 0 || left_angle > 180) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_task != NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    s_tuned_poses[pose] = (arm_pose_t) { right_angle, left_angle };
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t arm_action_tune_reset(void)
{
    if (s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_task != NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    for (int pose = ARM_TUNE_POSE_HOME; pose < ARM_TUNE_POSE_COUNT; ++pose) {
        s_tuned_poses[pose] = s_default_poses[pose];
    }
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

void arm_action_tune_print(void)
{
    if (s_mutex == NULL) {
        printf("Arm tuning: unavailable\n");
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    printf("Arm tuning (Servo 1=right, Servo 2=left):\n");
    for (int pose = ARM_TUNE_POSE_HOME; pose < ARM_TUNE_POSE_COUNT; ++pose) {
        const arm_pose_t *angles = &s_tuned_poses[pose];
        printf("  %-16s S1=%3d S2=%3d\n", tune_pose_name((arm_tune_pose_t)pose),
               angles->right_angle, angles->left_angle);
    }
    xSemaphoreGive(s_mutex);
}
