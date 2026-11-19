#include "speed_control.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "encoder.h"
#include "cps_log.h"
#include "motor_control.h"
#include "vofa_telemetry.h"

#define SPEED_CONTROL_PERIOD_MS 50
#define SPEED_CONTROL_TASK_PERIOD_MS 10
#define SPEED_CONTROL_START_PWM_PERCENT 12
#define SPEED_CONTROL_LOW_SPEED_MAX 50
/* Slower-wheel full-PWM speed, shared by both wheels for equal-speed commands. */
#define SPEED_CONTROL_MAX_CPS 5766
#define PID_KP_NUM 6
#define PID_KI_NUM 1
#define PID_KD_NUM 4
#define PID_GAIN_DENOMINATOR 2000
#define PID_LOW_KP_NUM 3
#define PID_LOW_KI_NUM 1
#define PID_LOW_KD_NUM 4
#define PID_LOW_GAIN_DENOMINATOR 2000
/* +/-20% PWM of trim: covers battery droop from 8.4 V down to ~7.0 V. */
#define PID_INTEGRAL_LIMIT 40000

static const char *TAG = "speed_pid";
static TaskHandle_t speed_control_task_handle;
static esp_timer_handle_t cps_log_timer;

typedef struct {
    int speed;
    int pwm_percent;
} feed_forward_point_t;

typedef struct {
    int set_speed;
    int target_cps;
    int feed_forward_percent;
    int measured_cps;
    int output_percent;
    int integral;
    int previous_measured_cps;
    int previous_count;
} wheel_pid_t;

typedef struct {
    bool active;
    int64_t previous_sample_us;
    wheel_pid_t left;
    wheel_pid_t right;
} speed_control_state_t;

static SemaphoreHandle_t control_mutex;
static speed_control_state_t state;

/*
 * Steady-state PWM measured on the real floor (BLE-driven, 8.36 V battery,
 * 2026-07-29) from open-loop cpslog sweeps at PWM 55..100 in both directions.
 * Left and right wheels matched within +/-0.5%, so one table per direction
 * serves both wheels. Below ~50% PWM the chassis stalls on the floor, so
 * speed <= 25 is outside the usable range and {25, 55} anchors the lowest
 * calibrated speed.
 */
static const feed_forward_point_t forward_feed_forward_points[] = {
    { 0, 0 }, { 25, 55 }, { 30, 56 }, { 40, 59 },
    { 60, 66 }, { 80, 77 }, { 100, 96 },
};
static const feed_forward_point_t reverse_feed_forward_points[] = {
    { 0, 0 }, { 25, 55 }, { 30, 55 }, { 40, 58 },
    { 60, 65 }, { 80, 75 }, { 100, 94 },
};
#define FEED_FORWARD_POINT_COUNT \
    (sizeof(forward_feed_forward_points) / sizeof(forward_feed_forward_points[0]))

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static void reset_wheel_pid(wheel_pid_t *wheel, int set_speed, int target_cps,
                            int feed_forward_percent, int current_count)
{
    wheel->set_speed = set_speed;
    wheel->target_cps = target_cps;
    wheel->feed_forward_percent = feed_forward_percent;
    wheel->measured_cps = 0;
    wheel->output_percent = 0;
    wheel->integral = 0;
    wheel->previous_measured_cps = 0;
    wheel->previous_count = current_count;
}

static bool is_low_speed_target(const wheel_pid_t *wheel)
{
    return abs(wheel->set_speed) <= SPEED_CONTROL_LOW_SPEED_MAX;
}

static int speed_to_cps(int speed, int full_speed_cps)
{
    if (speed < 0) {
        return -(((-speed * full_speed_cps) + SPEED_CONTROL_SPEED_MAX / 2) /
                 SPEED_CONTROL_SPEED_MAX);
    }
    return ((speed * full_speed_cps) + SPEED_CONTROL_SPEED_MAX / 2) /
           SPEED_CONTROL_SPEED_MAX;
}

static int feed_forward_percent(int speed, const feed_forward_point_t *points,
                                size_t point_count)
{
    int magnitude = speed < 0 ? -speed : speed;
    int output = points[point_count - 1].pwm_percent;

    for (size_t i = 1; i < point_count; ++i) {
        if (magnitude <= points[i].speed) {
            const feed_forward_point_t *lower = &points[i - 1];
            const feed_forward_point_t *upper = &points[i];
            int span = upper->speed - lower->speed;
            output = lower->pwm_percent +
                     ((magnitude - lower->speed) *
                      (upper->pwm_percent - lower->pwm_percent) + span / 2) / span;
            break;
        }
    }
    return speed < 0 ? -output : output;
}

static int initial_output_percent(int speed, int feed_forward)
{
    if (speed != 0 && abs(feed_forward) < SPEED_CONTROL_START_PWM_PERCENT) {
        return speed > 0 ? SPEED_CONTROL_START_PWM_PERCENT :
                           -SPEED_CONTROL_START_PWM_PERCENT;
    }
    return feed_forward;
}

static int update_wheel_pid(wheel_pid_t *wheel, int target_cps, int current_count,
                            int64_t elapsed_us)
{
    int64_t count_delta = (int64_t)current_count - wheel->previous_count;
    wheel->previous_count = current_count;
    wheel->measured_cps = (int)((count_delta * 1000000) / elapsed_us);

    int error = target_cps - wheel->measured_cps;
    bool low_speed = is_low_speed_target(wheel);
    int integral_candidate = clamp_int(wheel->integral + error,
                                       -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
    /* The derivative is applied to measured speed so a target step has no kick. */
    int measured_delta = wheel->measured_cps - wheel->previous_measured_cps;
    int gain_denominator = low_speed ? PID_LOW_GAIN_DENOMINATOR : PID_GAIN_DENOMINATOR;
    int kp = low_speed ? PID_LOW_KP_NUM : PID_KP_NUM;
    int ki = low_speed ? PID_LOW_KI_NUM : PID_KI_NUM;
    int kd = low_speed ? PID_LOW_KD_NUM : PID_KD_NUM;
    int64_t pid_sum = (int64_t)wheel->feed_forward_percent * gain_denominator +
                      (int64_t)kp * error +
                      (int64_t)ki * integral_candidate -
                      (int64_t)kd * measured_delta;
    int output_limit = MOTOR_SPEED_MAX;
    int output = clamp_int((int)(pid_sum / gain_denominator),
                           -output_limit, output_limit);

    if (!((output == output_limit && error > 0) ||
          (output == -output_limit && error < 0))) {
        wheel->integral = integral_candidate;
    }
    wheel->previous_measured_cps = wheel->measured_cps;
    wheel->output_percent = output;
    return output;
}

static void speed_control_task(void *arg)
{
    int previous_left_count = 0;
    int previous_right_count = 0;
    int64_t previous_telemetry_us = 0;

    while (true) {
        bool cps_log_sample_due = ulTaskNotifyTake(pdTRUE,
                                                    pdMS_TO_TICKS(SPEED_CONTROL_TASK_PERIOD_MS)) > 0;

        int64_t now_us = esp_timer_get_time();
        int left_count;
        int right_count;
        esp_err_t left_err = encoder_get_count(MOTOR_LEFT_INDEX + 1, &left_count);
        esp_err_t right_err = encoder_get_count(MOTOR_RIGHT_INDEX + 1, &right_count);

        xSemaphoreTake(control_mutex, portMAX_DELAY);
        if (left_err != ESP_OK || right_err != ESP_OK) {
            if (state.active) {
                state.active = false;
                motor_control_stop_all();
                ESP_LOGE(TAG, "Encoder read failed (left=%s right=%s); motors stopped",
                         esp_err_to_name(left_err), esp_err_to_name(right_err));
            }
            xSemaphoreGive(control_mutex);
            continue;
        }

        int set_left_speed = 0;
        int set_right_speed = 0;
        int pid_left_target_cps = 0;
        int pid_right_target_cps = 0;
        int left_pwm_percent = 0;
        int right_pwm_percent = 0;
        bool pid_active = state.active;
        if (state.active) {
            int64_t elapsed_us = now_us - state.previous_sample_us;
            if (elapsed_us >= SPEED_CONTROL_PERIOD_MS * 1000LL) {
                state.previous_sample_us = now_us;
                int left_output = update_wheel_pid(&state.left, state.left.target_cps,
                                                    left_count, elapsed_us);
                int right_output = update_wheel_pid(&state.right, state.right.target_cps,
                                                     right_count, elapsed_us);
                state.left.output_percent = left_output;
                state.right.output_percent = right_output;
                esp_err_t drive_err = motor_control_drive(state.left.output_percent,
                                                          state.right.output_percent);
                if (drive_err != ESP_OK) {
                    state.active = false;
                    motor_control_stop_all();
                    ESP_LOGE(TAG, "PID motor drive failed (%s); motors stopped",
                             esp_err_to_name(drive_err));
                }
            }

            if (state.active) {
                set_left_speed = state.left.set_speed;
                set_right_speed = state.right.set_speed;
                pid_left_target_cps = state.left.target_cps;
                pid_right_target_cps = state.right.target_cps;
                left_pwm_percent = state.left.output_percent;
                right_pwm_percent = state.right.output_percent;
            }
        }
        xSemaphoreGive(control_mutex);

        if (!pid_active) {
            (void)motor_control_get_speed(MOTOR_LEFT_INDEX + 1, &set_left_speed);
            (void)motor_control_get_speed(MOTOR_RIGHT_INDEX + 1, &set_right_speed);
            left_pwm_percent = set_left_speed;
            right_pwm_percent = set_right_speed;
        }

        int left_cps = 0;
        int right_cps = 0;
        if (cps_log_sample_due && previous_telemetry_us != 0) {
            int64_t elapsed_us = now_us - previous_telemetry_us;
            left_cps = (int)(((int64_t)left_count - previous_left_count) * 1000000 /
                             elapsed_us);
            right_cps = (int)(((int64_t)right_count - previous_right_count) * 1000000 /
                              elapsed_us);
        }
        if (cps_log_sample_due) {
            previous_left_count = left_count;
            previous_right_count = right_count;
            previous_telemetry_us = now_us;

            cps_log_append(now_us, set_left_speed, set_right_speed,
                           pid_left_target_cps, pid_right_target_cps,
                           left_cps, right_cps, left_pwm_percent, right_pwm_percent);

            vofa_telemetry_write(set_left_speed, set_right_speed,
                                 pid_left_target_cps, pid_right_target_cps,
                                 left_count, right_count, left_cps, right_cps,
                                 left_pwm_percent, right_pwm_percent);
        }
    }
}

static void cps_log_timer_callback(void *arg)
{
    xTaskNotifyGive((TaskHandle_t)arg);
}

esp_err_t speed_control_init(void)
{
    if (control_mutex != NULL) {
        return ESP_OK;
    }

    control_mutex = xSemaphoreCreateMutex();
    if (control_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t err = vofa_telemetry_init();
    if (err != ESP_OK) {
        vSemaphoreDelete(control_mutex);
        control_mutex = NULL;
        return err;
    }
    err = cps_log_init();
    if (err != ESP_OK) {
        vSemaphoreDelete(control_mutex);
        control_mutex = NULL;
        return err;
    }
    if (xTaskCreate(speed_control_task, "speed_pid", 3072, NULL, 5,
                    &speed_control_task_handle) != pdPASS) {
        vSemaphoreDelete(control_mutex);
        control_mutex = NULL;
        return ESP_ERR_NO_MEM;
    }

    const esp_timer_create_args_t cps_log_timer_args = {
        .callback = cps_log_timer_callback,
        .arg = speed_control_task_handle,
        .name = "cps_log",
    };
    err = esp_timer_create(&cps_log_timer_args, &cps_log_timer);
    if (err == ESP_OK) {
        err = esp_timer_start_periodic(cps_log_timer,
                                       CPS_LOG_SAMPLE_PERIOD_MS * 1000ULL);
    }
    if (err != ESP_OK) {
        if (cps_log_timer != NULL) {
            (void)esp_timer_delete(cps_log_timer);
            cps_log_timer = NULL;
        }
        vTaskDelete(speed_control_task_handle);
        speed_control_task_handle = NULL;
        vSemaphoreDelete(control_mutex);
        control_mutex = NULL;
        return err;
    }

    ESP_LOGI(TAG, "Dual-wheel PID ready (%d ms control, %d ms CPS samples)",
             SPEED_CONTROL_PERIOD_MS, CPS_LOG_SAMPLE_PERIOD_MS);
    return ESP_OK;
}

esp_err_t speed_control_set_target(int left_speed, int right_speed)
{
    if (control_mutex == NULL || left_speed < -SPEED_CONTROL_SPEED_MAX ||
        left_speed > SPEED_CONTROL_SPEED_MAX || right_speed < -SPEED_CONTROL_SPEED_MAX ||
        right_speed > SPEED_CONTROL_SPEED_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (left_speed == 0 && right_speed == 0) {
        speed_control_stop();
        return ESP_OK;
    }

    int left_target_cps = speed_to_cps(left_speed, SPEED_CONTROL_MAX_CPS);
    int right_target_cps = speed_to_cps(right_speed, SPEED_CONTROL_MAX_CPS);
    int left_feed_forward = feed_forward_percent(
        left_speed,
        left_speed < 0 ? reverse_feed_forward_points : forward_feed_forward_points,
        FEED_FORWARD_POINT_COUNT);
    int right_feed_forward = feed_forward_percent(
        right_speed,
        right_speed < 0 ? reverse_feed_forward_points : forward_feed_forward_points,
        FEED_FORWARD_POINT_COUNT);

    int left_count;
    int right_count;
    ESP_RETURN_ON_ERROR(encoder_get_count(MOTOR_LEFT_INDEX + 1, &left_count), TAG,
                        "read left encoder failed");
    ESP_RETURN_ON_ERROR(encoder_get_count(MOTOR_RIGHT_INDEX + 1, &right_count), TAG,
                        "read right encoder failed");

    xSemaphoreTake(control_mutex, portMAX_DELAY);
    reset_wheel_pid(&state.left, left_speed, left_target_cps, left_feed_forward,
                    left_count);
    reset_wheel_pid(&state.right, right_speed, right_target_cps, right_feed_forward,
                    right_count);
    state.previous_sample_us = esp_timer_get_time();
    state.active = true;
    state.left.output_percent = initial_output_percent(left_speed, left_feed_forward);
    state.right.output_percent = initial_output_percent(right_speed, right_feed_forward);
    esp_err_t err = motor_control_drive(state.left.output_percent,
                                        state.right.output_percent);
    if (err != ESP_OK) {
        state.active = false;
    }
    xSemaphoreGive(control_mutex);
    return err;
}

esp_err_t speed_control_update_target(int left_speed, int right_speed)
{
    if (control_mutex == NULL || left_speed < -SPEED_CONTROL_SPEED_MAX ||
        left_speed > SPEED_CONTROL_SPEED_MAX || right_speed < -SPEED_CONTROL_SPEED_MAX ||
        right_speed > SPEED_CONTROL_SPEED_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (left_speed == 0 && right_speed == 0) {
        speed_control_stop();
        return ESP_OK;
    }

    int left_target_cps = speed_to_cps(left_speed, SPEED_CONTROL_MAX_CPS);
    int right_target_cps = speed_to_cps(right_speed, SPEED_CONTROL_MAX_CPS);
    int left_feed_forward = feed_forward_percent(
        left_speed,
        left_speed < 0 ? reverse_feed_forward_points : forward_feed_forward_points,
        FEED_FORWARD_POINT_COUNT);
    int right_feed_forward = feed_forward_percent(
        right_speed,
        right_speed < 0 ? reverse_feed_forward_points : forward_feed_forward_points,
        FEED_FORWARD_POINT_COUNT);

    xSemaphoreTake(control_mutex, portMAX_DELAY);
    if (!state.active) {
        xSemaphoreGive(control_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    /* Keep encoder baselines and the accumulated wheel-speed correction intact. */
    state.left.set_speed = left_speed;
    state.left.target_cps = left_target_cps;
    state.left.feed_forward_percent = left_feed_forward;
    state.right.set_speed = right_speed;
    state.right.target_cps = right_target_cps;
    state.right.feed_forward_percent = right_feed_forward;
    xSemaphoreGive(control_mutex);
    return ESP_OK;
}

void speed_control_stop(void)
{
    if (control_mutex == NULL) {
        return;
    }

    xSemaphoreTake(control_mutex, portMAX_DELAY);
    state.active = false;
    motor_control_stop_all();
    xSemaphoreGive(control_mutex);
}

esp_err_t speed_control_get_telemetry(speed_control_telemetry_t *telemetry)
{
    if (telemetry == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (control_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(control_mutex, portMAX_DELAY);
    if (state.active) {
        telemetry->left_target_cps = state.left.target_cps;
        telemetry->right_target_cps = state.right.target_cps;
        telemetry->left_measured_cps = state.left.measured_cps;
        telemetry->right_measured_cps = state.right.measured_cps;
        telemetry->left_pwm_percent = state.left.output_percent;
        telemetry->right_pwm_percent = state.right.output_percent;
    } else {
        *telemetry = (speed_control_telemetry_t){ 0 };
    }
    xSemaphoreGive(control_mutex);
    return ESP_OK;
}

void speed_control_print_status(void)
{
    if (control_mutex == NULL) {
        printf("Speed PID: not initialized\n");
        return;
    }

    xSemaphoreTake(control_mutex, portMAX_DELAY);
    if (!state.active) {
        printf("Speed PID: idle\n");
    } else {
        printf("Speed PID: active; left set=%d target=%d cps measured=%d cps pwm=%d%%; "
               "right set=%d target=%d cps measured=%d cps pwm=%d%%\n",
               state.left.set_speed, state.left.target_cps, state.left.measured_cps,
               state.left.output_percent, state.right.set_speed, state.right.target_cps,
               state.right.measured_cps, state.right.output_percent);
    }
    xSemaphoreGive(control_mutex);
}
