#include "chassis_action.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

#include "arm_action.h"
#include "imu_lsm6ds3tr.h"
#include "imu_spin_log.h"
#include "speed_control.h"

/* Every chassis preset uses the calibrated speed/feed-forward/PID path, not raw PWM. */
#define CHASSIS_ACTION_SPEED_PERCENT 55
#define CHASSIS_ACTION_MOVE_DURATION_MS 1000
#define CHASSIS_ACTION_GREETING_DURATION_MS 1500
#define CHASSIS_ACTION_TURN_SPEED_PERCENT 70
#define CHASSIS_ACTION_TURN_DURATION_MS 1500
#define CHASSIS_ACTION_REVERSE_PARK_ARC_LEFT_SPEED_PERCENT -50
#define CHASSIS_ACTION_REVERSE_PARK_ARC_RIGHT_SPEED_PERCENT -100
#define CHASSIS_ACTION_REVERSE_PARK_ARC_TARGET_DEGREES 90.0f
#define CHASSIS_ACTION_REVERSE_PARK_ARC_TIMEOUT_MS 10000
#define CHASSIS_ACTION_REVERSE_PARK_TOTAL_DURATION_MS CHASSIS_ACTION_REVERSE_PARK_ARC_TIMEOUT_MS
/* Covers the phone player's start latency; keep in sync with ARM_REVERSE_PARK_LEAD_MS. */
#define CHASSIS_ACTION_REVERSE_PARK_LEAD_MS 600
#define CHASSIS_ACTION_SPIN_SPEED_PERCENT 100
#define CHASSIS_ACTION_SPIN_TIMEOUT_MS 10000
#define CHASSIS_ACTION_SPIN_TARGET_DEGREES 360.0f
#define CHASSIS_ACTION_SPIN_SAMPLE_PERIOD_MS 10
/* The close_arms preset needs about 700 ms to physically reach its home pose. */
#define CHASSIS_ACTION_HAPPY_ARM_HOME_LEAD_MS 700.0f
#define CHASSIS_ACTION_HAPPY_ARM_HOME_MIN_LEAD_DEGREES 50.0f
#define CHASSIS_ACTION_STRAIGHT_SAMPLE_PERIOD_MS 20
#define CHASSIS_ACTION_STRAIGHT_HEADING_KP_PERCENT_PER_DEG 1.5f
#define CHASSIS_ACTION_STRAIGHT_RATE_KD_PERCENT_PER_DPS 0.08f
#define CHASSIS_ACTION_STRAIGHT_MAX_CORRECTION_PERCENT 25.0f
#define CHASSIS_ACTION_GREETING_HEADING_RETURN_DEADBAND_DEGREES 3.0f
#define CHASSIS_ACTION_GREETING_HEADING_RETURN_TIMEOUT_MS 5000
#define CHASSIS_ACTION_EMOTION_SCARED_SPEED_PERCENT 80
#define CHASSIS_ACTION_EMOTION_SCARED_DURATION_MS 500
#define CHASSIS_ACTION_EMOTION_ANGRY_WOBBLE_SPEED_PERCENT 100
#define CHASSIS_ACTION_EMOTION_ANGRY_WOBBLE_DURATION_MS 200
#define CHASSIS_ACTION_EMOTION_ANGRY_WOBBLE_COUNT 2
#define CHASSIS_ACTION_CALIBRATION_SAMPLES 100
#define CHASSIS_ACTION_CALIBRATION_SAMPLE_PERIOD_MS 10
#define CHASSIS_ACTION_CALIBRATION_SETTLE_MS 1500
#define CHASSIS_ACTION_STATIONARY_SAMPLES 30
#define CHASSIS_ACTION_STATIONARY_TIMEOUT_MS 1500
#define CHASSIS_ACTION_STATIONARY_GYRO_MAX_RAW 600.0f
#define CHASSIS_ACTION_STATIONARY_ACCEL_MIN_NORM_RAW 12000.0f
#define CHASSIS_ACTION_STATIONARY_ACCEL_MAX_NORM_RAW 21000.0f
/* LSM6DS3TR-C configured at +/-500 dps: 17.5 mdps per raw LSB. */
#define CHASSIS_ACTION_GYRO_DPS_PER_LSB 0.0175f
/*
 * Happy-spin progress handling may cancel one arm task and start the home
 * task from inside the IMU turn loop.  2 KB overflows on ESP32-S3; reserve
 * 4 KB so this safety-critical path cannot reboot the controller.
 */
#define CHASSIS_ACTION_TASK_STACK_SIZE 4096
#define CHASSIS_ACTION_TASK_PRIORITY 4
#define CHASSIS_ACTION_CANCEL_TIMEOUT_MS 500

#define CHASSIS_ACTION_NVS_NAMESPACE "spin_imu"
#define CHASSIS_ACTION_NVS_KEY "calibration"
#define CHASSIS_ACTION_CALIBRATION_MAGIC 0x5350494EU /* "SPIN" */
#define CHASSIS_ACTION_CALIBRATION_VERSION 1U
#define CHASSIS_ACTION_TUNE_NVS_NAMESPACE "action_tune"
#define CHASSIS_ACTION_TUNE_NVS_KEY "reverse_park"
#define CHASSIS_ACTION_TUNE_MAGIC 0x5041524BU /* "PARK" */
#define CHASSIS_ACTION_TUNE_VERSION 1U

static const char *TAG = "chassis_action";

static SemaphoreHandle_t s_mutex;
static SemaphoreHandle_t s_completion;
static TaskHandle_t s_task;
static bool s_cancel_requested;
static bool s_calibration_active;
static chassis_action_id_t s_current_action;

typedef struct {
    uint32_t magic;
    uint32_t version;
    float gravity_x;
    float gravity_y;
    float gravity_z;
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;
} spin_imu_calibration_t;

static spin_imu_calibration_t s_spin_calibration;
static bool s_spin_calibrated;

typedef void (*turn_progress_callback_t)(float turned_degrees, float turn_rate_dps,
                                         float target_degrees, void *context);

typedef struct {
    bool home_started;
    bool home_attempted;
} happy_arm_home_sync_t;

typedef struct {
    bool enabled;
    int left_speed;
    int right_speed;
} speed_tune_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    int left_speed;
    int right_speed;
} reverse_park_speed_tune_t;

/* Per-action PID target overrides for onsite tuning. */
static speed_tune_t s_speed_tunes[CHASSIS_ACTION_COUNT];
static bool s_reverse_park_speed_tune_saved;

static const char *action_name(chassis_action_id_t action)
{
    switch (action) {
    case CHASSIS_ACTION_MOVE_FORWARD:
        return "move_forward";
    case CHASSIS_ACTION_MOVE_BACKWARD:
        return "move_backward";
    case CHASSIS_ACTION_GREETING:
        return "greeting";
    case CHASSIS_ACTION_TURN_LEFT:
        return "turn_left";
    case CHASSIS_ACTION_TURN_RIGHT:
        return "turn_right";
    case CHASSIS_ACTION_REVERSE_PARK:
        return "reverse_park";
    case CHASSIS_ACTION_SPIN_CLOCKWISE:
        return "spin_clockwise";
    case CHASSIS_ACTION_SPIN_COUNTERCLOCKWISE:
        return "spin_counterclockwise";
    case CHASSIS_ACTION_EMOTION_HAPPY:
        return "emotion_happy";
    case CHASSIS_ACTION_EMOTION_SCARED:
        return "emotion_scared";
    case CHASSIS_ACTION_EMOTION_ANGRY:
        return "emotion_angry";
    default:
        return "unknown";
    }
}

static bool action_is_spin(chassis_action_id_t action)
{
    return action == CHASSIS_ACTION_SPIN_CLOCKWISE ||
           action == CHASSIS_ACTION_SPIN_COUNTERCLOCKWISE ||
           action == CHASSIS_ACTION_EMOTION_HAPPY;
}

static bool action_is_heading_hold(chassis_action_id_t action)
{
    return action == CHASSIS_ACTION_MOVE_FORWARD || action == CHASSIS_ACTION_MOVE_BACKWARD ||
           action == CHASSIS_ACTION_GREETING;
}

static bool action_requires_imu(chassis_action_id_t action)
{
    return action_is_spin(action) || action_is_heading_hold(action) ||
           action == CHASSIS_ACTION_REVERSE_PARK;
}

static bool action_starts_arm(chassis_action_id_t action)
{
    return action == CHASSIS_ACTION_GREETING || action == CHASSIS_ACTION_REVERSE_PARK ||
           action == CHASSIS_ACTION_EMOTION_HAPPY ||
           action == CHASSIS_ACTION_EMOTION_SCARED || action == CHASSIS_ACTION_EMOTION_ANGRY;
}

static arm_action_id_t action_arm_preset(chassis_action_id_t action)
{
    switch (action) {
    case CHASSIS_ACTION_GREETING:
        return ARM_ACTION_WAVE_HAND;
    case CHASSIS_ACTION_EMOTION_HAPPY:
        return ARM_ACTION_EMOTION_HAPPY_SPIN;
    case CHASSIS_ACTION_REVERSE_PARK:
        return ARM_ACTION_REVERSE_PARK;
    case CHASSIS_ACTION_EMOTION_SCARED:
        return ARM_ACTION_EMOTION_SCARED;
    case CHASSIS_ACTION_EMOTION_ANGRY:
        return ARM_ACTION_EMOTION_ANGRY;
    default:
        return ARM_ACTION_HANDSHAKE;
    }
}

static bool action_is_valid(chassis_action_id_t action)
{
    return action >= CHASSIS_ACTION_MOVE_FORWARD &&
           action < CHASSIS_ACTION_COUNT;
}

static bool action_uses_primary_speed(chassis_action_id_t action)
{
    return action_is_valid(action) && action != CHASSIS_ACTION_EMOTION_ANGRY;
}

static void action_speed_percent(chassis_action_id_t action, int *left_speed, int *right_speed)
{
    switch (action) {
    case CHASSIS_ACTION_MOVE_FORWARD:
    case CHASSIS_ACTION_GREETING:
        *left_speed = CHASSIS_ACTION_SPEED_PERCENT;
        *right_speed = CHASSIS_ACTION_SPEED_PERCENT;
        break;
    case CHASSIS_ACTION_MOVE_BACKWARD:
        *left_speed = -CHASSIS_ACTION_SPEED_PERCENT;
        *right_speed = -CHASSIS_ACTION_SPEED_PERCENT;
        break;
    case CHASSIS_ACTION_TURN_LEFT:
        *left_speed = 0;
        *right_speed = CHASSIS_ACTION_TURN_SPEED_PERCENT;
        break;
    case CHASSIS_ACTION_TURN_RIGHT:
        *left_speed = CHASSIS_ACTION_TURN_SPEED_PERCENT;
        *right_speed = 0;
        break;
    case CHASSIS_ACTION_REVERSE_PARK:
        *left_speed = CHASSIS_ACTION_REVERSE_PARK_ARC_LEFT_SPEED_PERCENT;
        *right_speed = CHASSIS_ACTION_REVERSE_PARK_ARC_RIGHT_SPEED_PERCENT;
        break;
    case CHASSIS_ACTION_SPIN_CLOCKWISE:
    case CHASSIS_ACTION_EMOTION_HAPPY:
        *left_speed = CHASSIS_ACTION_SPIN_SPEED_PERCENT;
        *right_speed = -CHASSIS_ACTION_SPIN_SPEED_PERCENT;
        break;
    case CHASSIS_ACTION_SPIN_COUNTERCLOCKWISE:
        *left_speed = -CHASSIS_ACTION_SPIN_SPEED_PERCENT;
        *right_speed = CHASSIS_ACTION_SPIN_SPEED_PERCENT;
        break;
    case CHASSIS_ACTION_EMOTION_SCARED:
        *left_speed = -CHASSIS_ACTION_EMOTION_SCARED_SPEED_PERCENT;
        *right_speed = -CHASSIS_ACTION_EMOTION_SCARED_SPEED_PERCENT;
        break;
    case CHASSIS_ACTION_EMOTION_ANGRY:
        *left_speed = 0;
        *right_speed = 0;
        break;
    default:
        *left_speed = 0;
        *right_speed = 0;
        break;
    }

    if (action_is_valid(action) && s_speed_tunes[action].enabled) {
        *left_speed = s_speed_tunes[action].left_speed;
        *right_speed = s_speed_tunes[action].right_speed;
    }
}

static uint32_t action_duration_ms(chassis_action_id_t action)
{
    switch (action) {
    case CHASSIS_ACTION_GREETING:
        return CHASSIS_ACTION_GREETING_DURATION_MS;
    case CHASSIS_ACTION_TURN_LEFT:
    case CHASSIS_ACTION_TURN_RIGHT:
        return CHASSIS_ACTION_TURN_DURATION_MS;
    case CHASSIS_ACTION_REVERSE_PARK:
        return CHASSIS_ACTION_REVERSE_PARK_TOTAL_DURATION_MS;
    case CHASSIS_ACTION_EMOTION_SCARED:
        return CHASSIS_ACTION_EMOTION_SCARED_DURATION_MS;
    case CHASSIS_ACTION_EMOTION_ANGRY:
        return 2 * CHASSIS_ACTION_EMOTION_ANGRY_WOBBLE_COUNT *
               CHASSIS_ACTION_EMOTION_ANGRY_WOBBLE_DURATION_MS;
    default:
        return CHASSIS_ACTION_MOVE_DURATION_MS;
    }
}

static bool cancellation_requested(void)
{
    bool cancelled;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    cancelled = s_cancel_requested;
    xSemaphoreGive(s_mutex);
    return cancelled;
}

static bool spin_calibration_valid(const spin_imu_calibration_t *calibration)
{
    float gravity_norm = sqrtf(calibration->gravity_x * calibration->gravity_x +
                               calibration->gravity_y * calibration->gravity_y +
                               calibration->gravity_z * calibration->gravity_z);
    return calibration->magic == CHASSIS_ACTION_CALIBRATION_MAGIC &&
           calibration->version == CHASSIS_ACTION_CALIBRATION_VERSION &&
           isfinite(calibration->gravity_x) && isfinite(calibration->gravity_y) &&
           isfinite(calibration->gravity_z) && isfinite(calibration->gyro_bias_x) &&
           isfinite(calibration->gyro_bias_y) && isfinite(calibration->gyro_bias_z) &&
           gravity_norm > 0.98f && gravity_norm < 1.02f;
}

static void load_spin_calibration(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(CHASSIS_ACTION_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved IMU spin calibration; boot calibration is required");
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not open IMU spin calibration: %s", esp_err_to_name(err));
        return;
    }

    spin_imu_calibration_t calibration;
    size_t length = sizeof(calibration);
    err = nvs_get_blob(handle, CHASSIS_ACTION_NVS_KEY, &calibration, &length);
    nvs_close(handle);
    if (err != ESP_OK || length != sizeof(calibration) || !spin_calibration_valid(&calibration)) {
        ESP_LOGW(TAG, "Saved IMU spin calibration is invalid; a new calibration is required");
        return;
    }

    s_spin_calibration = calibration;
    s_spin_calibrated = true;
    ESP_LOGI(TAG, "Loaded saved IMU spin calibration");
}

static esp_err_t save_spin_calibration(const spin_imu_calibration_t *calibration)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(CHASSIS_ACTION_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(handle, CHASSIS_ACTION_NVS_KEY, calibration, sizeof(*calibration));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static bool reverse_park_speed_tune_valid(const reverse_park_speed_tune_t *tune)
{
    return tune->magic == CHASSIS_ACTION_TUNE_MAGIC &&
           tune->version == CHASSIS_ACTION_TUNE_VERSION &&
           tune->left_speed >= -100 && tune->left_speed <= 100 &&
           tune->right_speed >= -100 && tune->right_speed <= 100;
}

static void load_reverse_park_speed_tune(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(CHASSIS_ACTION_TUNE_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not open saved reverse_park speed: %s", esp_err_to_name(err));
        return;
    }

    reverse_park_speed_tune_t tune;
    size_t length = sizeof(tune);
    err = nvs_get_blob(handle, CHASSIS_ACTION_TUNE_NVS_KEY, &tune, &length);
    nvs_close(handle);
    if (err != ESP_OK || length != sizeof(tune) || !reverse_park_speed_tune_valid(&tune)) {
        ESP_LOGW(TAG, "Saved reverse_park speed is invalid; using firmware defaults");
        return;
    }

    s_speed_tunes[CHASSIS_ACTION_REVERSE_PARK] = (speed_tune_t) {
        .enabled = true,
        .left_speed = tune.left_speed,
        .right_speed = tune.right_speed,
    };
    s_reverse_park_speed_tune_saved = true;
    ESP_LOGI(TAG, "Loaded saved reverse_park PID targets: left=%d right=%d",
             tune.left_speed, tune.right_speed);
}

static esp_err_t save_reverse_park_speed_tune(const speed_tune_t *tune)
{
    reverse_park_speed_tune_t saved_tune = {
        .magic = CHASSIS_ACTION_TUNE_MAGIC,
        .version = CHASSIS_ACTION_TUNE_VERSION,
        .left_speed = tune->left_speed,
        .right_speed = tune->right_speed,
    };
    nvs_handle_t handle;
    esp_err_t err = nvs_open(CHASSIS_ACTION_TUNE_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(handle, CHASSIS_ACTION_TUNE_NVS_KEY, &saved_tune, sizeof(saved_tune));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static esp_err_t erase_reverse_park_speed_tune(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(CHASSIS_ACTION_TUNE_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_erase_key(handle, CHASSIS_ACTION_TUNE_NVS_KEY);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static esp_err_t wait_for_calibration_settle(void)
{
    uint32_t elapsed_ms = 0;
    while (elapsed_ms < CHASSIS_ACTION_CALIBRATION_SETTLE_MS) {
        if (cancellation_requested()) {
            return ESP_ERR_INVALID_STATE;
        }
        vTaskDelay(pdMS_TO_TICKS(CHASSIS_ACTION_CALIBRATION_SAMPLE_PERIOD_MS));
        elapsed_ms += CHASSIS_ACTION_CALIBRATION_SAMPLE_PERIOD_MS;
    }
    return ESP_OK;
}

static esp_err_t wait_for_imu_stationary(void)
{
    int stationary_samples = 0;
    int64_t deadline_us = esp_timer_get_time() +
                          CHASSIS_ACTION_STATIONARY_TIMEOUT_MS * 1000LL;

    while (esp_timer_get_time() < deadline_us) {
        if (cancellation_requested()) {
            return ESP_ERR_INVALID_STATE;
        }

        imu_lsm6ds3tr_sample_t sample;
        esp_err_t err = imu_read_sample(&sample);
        if (err != ESP_OK) {
            return err;
        }

        float gyro_x = sample.gyro_x;
        float gyro_y = sample.gyro_y;
        float gyro_z = sample.gyro_z;
        float accel_x = sample.accel_x;
        float accel_y = sample.accel_y;
        float accel_z = sample.accel_z;
        float gyro_norm = sqrtf(gyro_x * gyro_x + gyro_y * gyro_y + gyro_z * gyro_z);
        float accel_norm = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
        bool stationary = gyro_norm <= CHASSIS_ACTION_STATIONARY_GYRO_MAX_RAW &&
                          accel_norm >= CHASSIS_ACTION_STATIONARY_ACCEL_MIN_NORM_RAW &&
                          accel_norm <= CHASSIS_ACTION_STATIONARY_ACCEL_MAX_NORM_RAW;

        stationary_samples = stationary ? stationary_samples + 1 : 0;
        if (stationary_samples >= CHASSIS_ACTION_STATIONARY_SAMPLES) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(CHASSIS_ACTION_CALIBRATION_SAMPLE_PERIOD_MS));
    }
    return ESP_ERR_INVALID_STATE;
}

esp_err_t chassis_action_calibrate_imu(void)
{
    if (s_mutex == NULL || s_completion == NULL || !imu_is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (arm_action_is_active()) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_task != NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    while (xSemaphoreTake(s_completion, 0) == pdTRUE) {
    }
    s_task = xTaskGetCurrentTaskHandle();
    s_calibration_active = true;
    s_cancel_requested = false;
    xSemaphoreGive(s_mutex);

    /* The PCB is fixed but tilted. Average its stationary acceleration to learn
     * the gravity direction in sensor coordinates and refresh the three-axis
     * gyro zero bias on every boot; bias changes with temperature. Future spin
     * commands project all three gyro axes onto this saved gravity direction. */
    /* speed_control_stop() also drives both GC9112 inputs low. */
    speed_control_stop();
    float accel_x_sum = 0.0f;
    float accel_y_sum = 0.0f;
    float accel_z_sum = 0.0f;
    float gyro_x_sum = 0.0f;
    float gyro_y_sum = 0.0f;
    float gyro_z_sum = 0.0f;
    ESP_LOGI(TAG, "Motor outputs low; waiting %u ms for IMU calibration settle",
             (unsigned)CHASSIS_ACTION_CALIBRATION_SETTLE_MS);
    esp_err_t err = wait_for_calibration_settle();
    if (err == ESP_OK) {
        err = wait_for_imu_stationary();
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "IMU stationary check passed; collecting %d bias samples",
                 CHASSIS_ACTION_CALIBRATION_SAMPLES);
    } else if (!cancellation_requested()) {
        ESP_LOGW(TAG, "IMU did not remain stationary; keeping previous calibration");
    }

    for (int i = 0; err == ESP_OK && i < CHASSIS_ACTION_CALIBRATION_SAMPLES; ++i) {
        if (cancellation_requested()) {
            err = ESP_ERR_INVALID_STATE;
            break;
        }

        imu_lsm6ds3tr_sample_t sample;
        err = imu_read_sample(&sample);
        if (err != ESP_OK) {
            break;
        }
        accel_x_sum += sample.accel_x;
        accel_y_sum += sample.accel_y;
        accel_z_sum += sample.accel_z;
        gyro_x_sum += sample.gyro_x;
        gyro_y_sum += sample.gyro_y;
        gyro_z_sum += sample.gyro_z;
        if (i + 1 < CHASSIS_ACTION_CALIBRATION_SAMPLES) {
            vTaskDelay(pdMS_TO_TICKS(CHASSIS_ACTION_CALIBRATION_SAMPLE_PERIOD_MS));
        }
    }

    spin_imu_calibration_t calibration = {
        .magic = CHASSIS_ACTION_CALIBRATION_MAGIC,
        .version = CHASSIS_ACTION_CALIBRATION_VERSION,
    };
    if (err == ESP_OK) {
        float gravity_norm = sqrtf(accel_x_sum * accel_x_sum + accel_y_sum * accel_y_sum +
                                   accel_z_sum * accel_z_sum);
        if (gravity_norm < 1.0f) {
            err = ESP_ERR_INVALID_RESPONSE;
        } else {
            calibration.gravity_x = accel_x_sum / gravity_norm;
            calibration.gravity_y = accel_y_sum / gravity_norm;
            calibration.gravity_z = accel_z_sum / gravity_norm;
            calibration.gyro_bias_x = gyro_x_sum / CHASSIS_ACTION_CALIBRATION_SAMPLES;
            calibration.gyro_bias_y = gyro_y_sum / CHASSIS_ACTION_CALIBRATION_SAMPLES;
            calibration.gyro_bias_z = gyro_z_sum / CHASSIS_ACTION_CALIBRATION_SAMPLES;
            err = save_spin_calibration(&calibration);
        }
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (err == ESP_OK) {
        s_spin_calibration = calibration;
        s_spin_calibrated = true;
    }
    s_task = NULL;
    s_calibration_active = false;
    s_cancel_requested = false;
    xSemaphoreGive(s_mutex);
    xSemaphoreGive(s_completion);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "IMU spin calibration saved (100 stationary samples)");
    }
    return err;
}

static esp_err_t run_turn_until_target(float target_degrees, uint32_t timeout_ms,
                                       float *measured_degrees, float *peak_turn_rate_dps,
                                       turn_progress_callback_t progress_callback, void *context)
{
    spin_imu_calibration_t calibration;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool calibrated = s_spin_calibrated;
    calibration = s_spin_calibration;
    xSemaphoreGive(s_mutex);
    if (!calibrated) {
        return ESP_ERR_INVALID_STATE;
    }

    float turned_degrees = 0.0f;
    float peak_rate_dps = 0.0f;
    int64_t previous_us = esp_timer_get_time();
    int64_t deadline_us = previous_us + timeout_ms * 1000LL;
    esp_err_t err = ESP_OK;

    while (esp_timer_get_time() < deadline_us) {
        if (cancellation_requested()) {
            return ESP_ERR_INVALID_STATE;
        }

        imu_lsm6ds3tr_sample_t sample;
        err = imu_read_sample(&sample);
        if (err != ESP_OK) {
            return err;
        }

        int64_t now_us = esp_timer_get_time();
        float elapsed_seconds = (float)(now_us - previous_us) / 1000000.0f;
        previous_us = now_us;
        float vertical_rate_raw = (sample.gyro_x - calibration.gyro_bias_x) * calibration.gravity_x +
                                  (sample.gyro_y - calibration.gyro_bias_y) * calibration.gravity_y +
                                  (sample.gyro_z - calibration.gyro_bias_z) * calibration.gravity_z;
        float vertical_rate_dps = vertical_rate_raw * CHASSIS_ACTION_GYRO_DPS_PER_LSB;
        if (fabsf(vertical_rate_dps) > peak_rate_dps) {
            peak_rate_dps = fabsf(vertical_rate_dps);
        }
        turned_degrees += vertical_rate_dps * elapsed_seconds;
        speed_control_telemetry_t speed_telemetry;
        err = speed_control_get_telemetry(&speed_telemetry);
        if (err != ESP_OK) {
            return err;
        }
        imu_spin_log_append(now_us, &sample, vertical_rate_dps, turned_degrees,
                            &speed_telemetry);
        if (progress_callback != NULL) {
            progress_callback(turned_degrees, vertical_rate_dps, target_degrees, context);
        }
        if (fabsf(turned_degrees) >= fabsf(target_degrees)) {
            *measured_degrees = turned_degrees;
            *peak_turn_rate_dps = peak_rate_dps;
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(CHASSIS_ACTION_SPIN_SAMPLE_PERIOD_MS));
    }

    *measured_degrees = turned_degrees;
    *peak_turn_rate_dps = peak_rate_dps;
    return ESP_ERR_TIMEOUT;
}

static esp_err_t run_straight_heading_hold(int left_base_speed, int right_base_speed,
                                           uint32_t duration_ms,
                                           float *final_heading_degrees,
                                           float *peak_heading_degrees)
{
    spin_imu_calibration_t calibration;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool calibrated = s_spin_calibrated;
    calibration = s_spin_calibration;
    xSemaphoreGive(s_mutex);
    if (!calibrated) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = speed_control_set_target(left_base_speed, right_base_speed);
    if (err != ESP_OK) {
        return err;
    }

    float heading_degrees = 0.0f;
    float peak_heading = 0.0f;
    int previous_left_speed = left_base_speed;
    int previous_right_speed = right_base_speed;
    int64_t previous_us = esp_timer_get_time();
    int64_t deadline_us = previous_us + duration_ms * 1000LL;

    while (esp_timer_get_time() < deadline_us) {
        if (cancellation_requested()) {
            return ESP_ERR_INVALID_STATE;
        }

        imu_lsm6ds3tr_sample_t sample;
        err = imu_read_sample(&sample);
        if (err != ESP_OK) {
            return err;
        }

        int64_t now_us = esp_timer_get_time();
        float elapsed_seconds = (float)(now_us - previous_us) / 1000000.0f;
        previous_us = now_us;
        float vertical_rate_raw =
            (sample.gyro_x - calibration.gyro_bias_x) * calibration.gravity_x +
            (sample.gyro_y - calibration.gyro_bias_y) * calibration.gravity_y +
            (sample.gyro_z - calibration.gyro_bias_z) * calibration.gravity_z;
        float vertical_rate_dps = vertical_rate_raw * CHASSIS_ACTION_GYRO_DPS_PER_LSB;
        heading_degrees += vertical_rate_dps * elapsed_seconds;
        if (fabsf(heading_degrees) > peak_heading) {
            peak_heading = fabsf(heading_degrees);
        }

        /* Positive projected yaw is the existing clockwise spin convention. */
        float correction = heading_degrees * CHASSIS_ACTION_STRAIGHT_HEADING_KP_PERCENT_PER_DEG +
                           vertical_rate_dps * CHASSIS_ACTION_STRAIGHT_RATE_KD_PERCENT_PER_DPS;
        correction = fmaxf(-CHASSIS_ACTION_STRAIGHT_MAX_CORRECTION_PERCENT,
                           fminf(correction, CHASSIS_ACTION_STRAIGHT_MAX_CORRECTION_PERCENT));
        int correction_percent = (int)lroundf(correction);
        int left_speed = left_base_speed - correction_percent;
        int right_speed = right_base_speed + correction_percent;
        if (left_speed > 100) {
            left_speed = 100;
        } else if (left_speed < -100) {
            left_speed = -100;
        }
        if (right_speed > 100) {
            right_speed = 100;
        } else if (right_speed < -100) {
            right_speed = -100;
        }

        if (left_speed != previous_left_speed || right_speed != previous_right_speed) {
            err = speed_control_update_target(left_speed, right_speed);
            if (err != ESP_OK) {
                return err;
            }
            previous_left_speed = left_speed;
            previous_right_speed = right_speed;
        }
        vTaskDelay(pdMS_TO_TICKS(CHASSIS_ACTION_STRAIGHT_SAMPLE_PERIOD_MS));
    }

    *final_heading_degrees = heading_degrees;
    *peak_heading_degrees = peak_heading;
    return ESP_OK;
}

static esp_err_t run_greeting_heading_return(float heading_error_degrees)
{
    if (fabsf(heading_error_degrees) < CHASSIS_ACTION_GREETING_HEADING_RETURN_DEADBAND_DEGREES) {
        ESP_LOGI(TAG, "greeting heading drift %.1f deg is within the deadband",
                 (double)heading_error_degrees);
        return ESP_OK;
    }

    /* Positive heading is clockwise, so turn in the opposite direction. */
    int turn_speed = heading_error_degrees > 0.0f ? -CHASSIS_ACTION_TURN_SPEED_PERCENT
                                                  : CHASSIS_ACTION_TURN_SPEED_PERCENT;
    esp_err_t err = speed_control_set_target(turn_speed, -turn_speed);
    if (err != ESP_OK) {
        return err;
    }

    float returned_degrees = 0.0f;
    float peak_turn_rate_dps = 0.0f;
    err = run_turn_until_target(fabsf(heading_error_degrees),
                                CHASSIS_ACTION_GREETING_HEADING_RETURN_TIMEOUT_MS,
                                &returned_degrees, &peak_turn_rate_dps, NULL, NULL);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "greeting heading restored: forward drift %.1f deg, shortest return %.1f deg",
                 (double)heading_error_degrees, (double)returned_degrees);
    }
    return err;
}

static esp_err_t run_speed_segment(int left_speed, int right_speed, uint32_t duration_ms)
{
    esp_err_t err = speed_control_set_target(left_speed, right_speed);
    if (err != ESP_OK) {
        return err;
    }

    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(duration_ms)) != 0 ||
        cancellation_requested()) {
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

typedef struct {
    bool reverse;
    uint32_t duration_ms;
} reverse_park_segment_t;

/*
 * "请注意倒车8s": 136 BPM, one bar is about 1765 ms and the audible BGM ends
 * at ~8.4 s. The fan reverse runs only on odd bars; even bars are hold-and-
 * check pauses, so the manoeuvre spans the whole music and the wheels stop
 * right as it fades. If the IMU target completes early, the chassis holds
 * position for the rest of the timeline instead of running past it.
 */
static const reverse_park_segment_t reverse_park_segments[] = {
    { false, CHASSIS_ACTION_REVERSE_PARK_LEAD_MS },
    { true,  1765 },  /* bar 1: reverse */
    { false, 1765 },  /* bar 2: hold */
    { true,  1766 },  /* bar 3: reverse */
    { false, 1765 },  /* bar 4: hold */
    { true,  1226 },  /* bar 5: reverse into the music end */
};

static esp_err_t wait_reverse_park_segment(uint32_t duration_ms)
{
    speed_control_stop();
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(duration_ms)) != 0 ||
        cancellation_requested()) {
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static esp_err_t run_reverse_park(void)
{
    int left_speed;
    int right_speed;
    action_speed_percent(CHASSIS_ACTION_REVERSE_PARK, &left_speed, &right_speed);

    /* Both wheels reverse, with the right wheel faster, to trace one quadrant
     * of a fan-shaped reversing manoeuvre without pivoting in place. */
    float arc_degrees = 0.0f;
    float arc_peak_rate_dps = 0.0f;
    bool target_reached = false;

    for (size_t seg = 0; seg < sizeof(reverse_park_segments) / sizeof(reverse_park_segments[0]);
         ++seg) {
        const reverse_park_segment_t *segment = &reverse_park_segments[seg];
        if (!segment->reverse || target_reached) {
            esp_err_t err = wait_reverse_park_segment(segment->duration_ms);
            if (err != ESP_OK) {
                return err;
            }
            continue;
        }

        int64_t segment_start_us = esp_timer_get_time();
        esp_err_t err = speed_control_set_target(left_speed, right_speed);
        if (err != ESP_OK) {
            return err;
        }
        float segment_degrees = 0.0f;
        err = run_turn_until_target(CHASSIS_ACTION_REVERSE_PARK_ARC_TARGET_DEGREES -
                                    fabsf(arc_degrees),
                                    segment->duration_ms, &segment_degrees,
                                    &arc_peak_rate_dps, NULL, NULL);
        arc_degrees += fabsf(segment_degrees);
        if (err == ESP_ERR_TIMEOUT) {
            continue;  /* segment time elapsed: the next bar drives on */
        }
        if (err != ESP_OK) {
            return err;
        }
        target_reached = true;
        int64_t remaining_us = (int64_t)segment->duration_ms * 1000 -
                               (esp_timer_get_time() - segment_start_us);
        if (remaining_us > 1000) {
            err = wait_reverse_park_segment((uint32_t)(remaining_us / 1000));
            if (err != ESP_OK) {
                return err;
            }
        }
    }

    speed_control_stop();
    ESP_LOGI(TAG, "reverse_park complete: BGM-timed fan-shaped reverse %d/%d %.1f deg (peak %.1f dps)",
             left_speed, right_speed, (double)arc_degrees, (double)arc_peak_rate_dps);
    return ESP_OK;
}

static esp_err_t start_happy_arm_home(happy_arm_home_sync_t *sync)
{
    if (sync->home_started) {
        return ESP_OK;
    }

    sync->home_attempted = true;
    esp_err_t err = arm_action_cancel();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "emotion_happy could not stop arm wave: %s", esp_err_to_name(err));
        return err;
    }

    err = arm_action_start(ARM_ACTION_EMOTION_HAPPY_HOME);
    if (err == ESP_OK) {
        sync->home_started = true;
    } else {
        ESP_LOGW(TAG, "emotion_happy could not start arm home: %s", esp_err_to_name(err));
    }
    return err;
}

static void start_happy_arm_home_near_spin_end(float turned_degrees, float turn_rate_dps,
                                                float target_degrees, void *context)
{
    happy_arm_home_sync_t *sync = context;
    if (sync == NULL || sync->home_attempted) {
        return;
    }

    float remaining_degrees = fabsf(target_degrees) - fabsf(turned_degrees);
    float lead_degrees = fabsf(turn_rate_dps) *
                         (CHASSIS_ACTION_HAPPY_ARM_HOME_LEAD_MS / 1000.0f);
    if (lead_degrees < CHASSIS_ACTION_HAPPY_ARM_HOME_MIN_LEAD_DEGREES) {
        lead_degrees = CHASSIS_ACTION_HAPPY_ARM_HOME_MIN_LEAD_DEGREES;
    }
    if (remaining_degrees <= lead_degrees) {
        (void)start_happy_arm_home(sync);
    }
}

static esp_err_t run_emotion_angry_wobble(void)
{
    for (int wobble = 0; wobble < CHASSIS_ACTION_EMOTION_ANGRY_WOBBLE_COUNT; ++wobble) {
        esp_err_t err = run_speed_segment(CHASSIS_ACTION_EMOTION_ANGRY_WOBBLE_SPEED_PERCENT,
                                          -CHASSIS_ACTION_EMOTION_ANGRY_WOBBLE_SPEED_PERCENT,
                                          CHASSIS_ACTION_EMOTION_ANGRY_WOBBLE_DURATION_MS);
        if (err != ESP_OK) {
            return err;
        }
        err = run_speed_segment(-CHASSIS_ACTION_EMOTION_ANGRY_WOBBLE_SPEED_PERCENT,
                                CHASSIS_ACTION_EMOTION_ANGRY_WOBBLE_SPEED_PERCENT,
                                CHASSIS_ACTION_EMOTION_ANGRY_WOBBLE_DURATION_MS);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

static void chassis_action_task(void *arg)
{
    chassis_action_id_t action = (chassis_action_id_t)(uintptr_t)arg;
    int left_speed;
    int right_speed;
    action_speed_percent(action, &left_speed, &right_speed);
    uint32_t duration_ms = action_duration_ms(action);
    bool completed = false;
    bool arm_started = false;
    float spin_degrees = 0.0f;
    float spin_peak_rate_dps = 0.0f;
    bool spin_log_started = false;
    happy_arm_home_sync_t happy_arm_home = { 0 };
    float straight_heading_degrees = 0.0f;
    float straight_peak_heading_degrees = 0.0f;

    if (!cancellation_requested()) {
        esp_err_t err = ESP_OK;
        if (action_starts_arm(action)) {
            /* The arm preset and chassis motion start from this same task. */
            err = arm_action_start(action_arm_preset(action));
            arm_started = err == ESP_OK;
            if (err != ESP_OK && action == CHASSIS_ACTION_REVERSE_PARK) {
                /* Arm choreography is decorative; never block the established parking path. */
                ESP_LOGW(TAG, "reverse_park arms unavailable: %s; continuing chassis action",
                         esp_err_to_name(err));
                err = ESP_OK;
            }
        }
        if (err == ESP_OK && action == CHASSIS_ACTION_REVERSE_PARK) {
            err = run_reverse_park();
            completed = err == ESP_OK && !cancellation_requested();
        } else if (err == ESP_OK && action == CHASSIS_ACTION_EMOTION_ANGRY) {
            err = run_emotion_angry_wobble();
            completed = err == ESP_OK && !cancellation_requested();
        } else if (err == ESP_OK && action_is_heading_hold(action)) {
            err = run_straight_heading_hold(left_speed, right_speed, duration_ms,
                                            &straight_heading_degrees,
                                            &straight_peak_heading_degrees);
            if (err == ESP_OK && action == CHASSIS_ACTION_GREETING) {
                err = run_greeting_heading_return(straight_heading_degrees);
            }
            completed = err == ESP_OK && !cancellation_requested();
        } else if (err == ESP_OK) {
            err = speed_control_set_target(left_speed, right_speed);
            if (err == ESP_OK && action_is_spin(action)) {
                esp_err_t log_err = imu_spin_log_start(left_speed, right_speed);
                spin_log_started = log_err == ESP_OK;
                if (log_err != ESP_OK) {
                    ESP_LOGW(TAG, "%s IMU log disabled: %s", action_name(action),
                             esp_err_to_name(log_err));
                }
                turn_progress_callback_t progress_callback =
                    action == CHASSIS_ACTION_EMOTION_HAPPY && arm_started
                        ? start_happy_arm_home_near_spin_end
                        : NULL;
                err = run_turn_until_target(CHASSIS_ACTION_SPIN_TARGET_DEGREES,
                                             CHASSIS_ACTION_SPIN_TIMEOUT_MS,
                                             &spin_degrees, &spin_peak_rate_dps,
                                             progress_callback, &happy_arm_home);
                completed = err == ESP_OK && !cancellation_requested();
                if (err != ESP_OK && !cancellation_requested()) {
                    ESP_LOGE(TAG, "%s ended at %.1f deg: %s", action_name(action),
                             (double)spin_degrees, esp_err_to_name(err));
                }
            } else if (err == ESP_OK) {
                /* A task notification arrives immediately when stop/manual control cancels the preset. */
                completed = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(duration_ms)) == 0 &&
                            !cancellation_requested();
            }
        }
        if (err != ESP_OK && !cancellation_requested()) {
            ESP_LOGE(TAG, "%s start failed: %s", action_name(action), esp_err_to_name(err));
        }
    }

    /* Always clear the speed controller at the terminal state, including BLE or manual cancel. */
    speed_control_stop();

    if (spin_log_started) {
        size_t sample_count = 0;
        unsigned int session_id = 0;
        esp_err_t log_err = imu_spin_log_stop_and_save(&sample_count, &session_id);
        if (log_err == ESP_OK) {
            ESP_LOGI(TAG, "%s IMU log saved as ID %03u: %u samples", action_name(action),
                     session_id, (unsigned)sample_count);
        } else {
            ESP_LOGW(TAG, "%s IMU log save failed: %s", action_name(action),
                     esp_err_to_name(log_err));
        }
    }

    /* The arm begins its 700 ms home transition before the last part of the spin. */
    if (action == CHASSIS_ACTION_EMOTION_HAPPY && arm_started) {
        if (completed && !happy_arm_home.home_started && !cancellation_requested()) {
            (void)start_happy_arm_home(&happy_arm_home);
        } else if (!completed) {
            (void)arm_action_cancel();
        }
        arm_started = false;
    }

    bool cancelled = cancellation_requested();
    if (action_starts_arm(action) && arm_started && !completed) {
        (void)arm_action_cancel();
    }
    if (completed) {
        if (action == CHASSIS_ACTION_REVERSE_PARK) {
            ESP_LOGI(TAG, "%s complete: -50/-100 fan-shaped reverse to IMU 90 degrees",
                     action_name(action));
        } else if (action_is_spin(action)) {
            ESP_LOGI(TAG, "%s complete: IMU gravity-axis turn %.1f deg, peak %.1f dps",
                     action_name(action), (double)spin_degrees, (double)spin_peak_rate_dps);
        } else if (action_is_heading_hold(action)) {
            ESP_LOGI(TAG, "%s complete: IMU heading hold %.1f deg final, %.1f deg peak",
                     action_name(action), (double)straight_heading_degrees,
                     (double)straight_peak_heading_degrees);
        } else if (action == CHASSIS_ACTION_EMOTION_ANGRY) {
            ESP_LOGI(TAG, "%s complete: %d alternating PID wobble segments",
                     action_name(action), 2 * CHASSIS_ACTION_EMOTION_ANGRY_WOBBLE_COUNT);
        } else {
            ESP_LOGI(TAG, "%s complete: PID left=%d right=%d for %u ms", action_name(action),
                     left_speed, right_speed, (unsigned)duration_ms);
        }
    } else if (cancelled) {
        ESP_LOGI(TAG, "%s cancelled; motors stopped", action_name(action));
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_task = NULL;
    s_cancel_requested = false;
    xSemaphoreGive(s_mutex);
    xSemaphoreGive(s_completion);
    vTaskDelete(NULL);
}

esp_err_t chassis_action_init(void)
{
    if (s_mutex != NULL) {
        return ESP_OK;
    }

    s_mutex = xSemaphoreCreateMutex();
    s_completion = xSemaphoreCreateBinary();
    if (s_mutex == NULL || s_completion == NULL) {
        return ESP_ERR_NO_MEM;
    }
    load_spin_calibration();
    load_reverse_park_speed_tune();
    return ESP_OK;
}

esp_err_t chassis_action_start(chassis_action_id_t action)
{
    if (s_mutex == NULL || s_completion == NULL || !action_is_valid(action)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (action_starts_arm(action) && arm_action_is_active()) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_task != NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (action_requires_imu(action) && !s_spin_calibrated) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    while (xSemaphoreTake(s_completion, 0) == pdTRUE) {
    }

    s_current_action = action;
    s_cancel_requested = false;
    if (xTaskCreate(chassis_action_task, action_name(action), CHASSIS_ACTION_TASK_STACK_SIZE,
                    (void *)(uintptr_t)action, CHASSIS_ACTION_TASK_PRIORITY, &s_task) != pdPASS) {
        s_task = NULL;
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t chassis_action_cancel(void)
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

    /* Stop now; the task also stops on its way out, but never let it coast for a wait period. */
    speed_control_stop();
    return xSemaphoreTake(s_completion, pdMS_TO_TICKS(CHASSIS_ACTION_CANCEL_TIMEOUT_MS)) == pdTRUE
               ? ESP_OK
               : ESP_ERR_TIMEOUT;
}

bool chassis_action_is_active(void)
{
    if (s_mutex == NULL) {
        return false;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool active = s_task != NULL;
    xSemaphoreGive(s_mutex);
    return active;
}

esp_err_t chassis_action_read_calibrated_yaw_rate(float *rate_dps)
{
    if (rate_dps == NULL || s_mutex == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool calibrated = s_spin_calibrated;
    spin_imu_calibration_t calibration = s_spin_calibration;
    xSemaphoreGive(s_mutex);
    if (!calibrated) {
        return ESP_ERR_INVALID_STATE;
    }

    imu_lsm6ds3tr_sample_t sample;
    esp_err_t err = imu_read_sample(&sample);
    if (err != ESP_OK) {
        return err;
    }

    float vertical_rate_raw =
        (sample.gyro_x - calibration.gyro_bias_x) * calibration.gravity_x +
        (sample.gyro_y - calibration.gyro_bias_y) * calibration.gravity_y +
        (sample.gyro_z - calibration.gyro_bias_z) * calibration.gravity_z;
    *rate_dps = vertical_rate_raw * CHASSIS_ACTION_GYRO_DPS_PER_LSB;
    return ESP_OK;
}

void chassis_action_print_status(void)
{
    if (s_mutex == NULL) {
        printf("Chassis action: unavailable\n");
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_task == NULL) {
        printf("Chassis action: idle\n");
    } else if (s_calibration_active) {
        printf("Chassis action: calibrating IMU for spin\n");
    } else {
        int left_speed;
        int right_speed;
        action_speed_percent(s_current_action, &left_speed, &right_speed);
        if (s_current_action == CHASSIS_ACTION_EMOTION_ANGRY) {
            printf("Chassis action: %s%s (PID left/right alternates %d%% for %u ms)\n",
                   action_name(s_current_action), s_cancel_requested ? " (cancelling)" : "",
                   CHASSIS_ACTION_EMOTION_ANGRY_WOBBLE_SPEED_PERCENT,
                   (unsigned)action_duration_ms(s_current_action));
        } else if (action_is_spin(s_current_action)) {
            printf("Chassis action: %s%s (PID left=%d%%, right=%d%%, IMU 360 deg)\n",
                   action_name(s_current_action), s_cancel_requested ? " (cancelling)" : "",
                   left_speed, right_speed);
        } else {
            printf("Chassis action: %s%s (PID left=%d%%, right=%d%%, %u ms)\n",
                   action_name(s_current_action), s_cancel_requested ? " (cancelling)" : "",
                   left_speed, right_speed, (unsigned)action_duration_ms(s_current_action));
        }
    }
    printf("Spin IMU calibration: %s\n", s_spin_calibrated ? "saved" : "required (imu calibrate)");
    xSemaphoreGive(s_mutex);
}

esp_err_t chassis_action_tune_set_speed(chassis_action_id_t action, int left_speed,
                                        int right_speed)
{
    if (s_mutex == NULL || !action_uses_primary_speed(action) || left_speed < -100 ||
        left_speed > 100 || right_speed < -100 || right_speed > 100) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_task != NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    s_speed_tunes[action] = (speed_tune_t) {
        .enabled = true,
        .left_speed = left_speed,
        .right_speed = right_speed,
    };
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t chassis_action_tune_save_reverse_park(void)
{
    if (s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_task != NULL || !s_speed_tunes[CHASSIS_ACTION_REVERSE_PARK].enabled) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    speed_tune_t tune = s_speed_tunes[CHASSIS_ACTION_REVERSE_PARK];
    esp_err_t err = save_reverse_park_speed_tune(&tune);
    if (err == ESP_OK) {
        s_reverse_park_speed_tune_saved = true;
    }
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t chassis_action_tune_reset(void)
{
    if (s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_task != NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    for (int action = CHASSIS_ACTION_MOVE_FORWARD; action < CHASSIS_ACTION_COUNT; ++action) {
        s_speed_tunes[action].enabled = false;
    }
    esp_err_t err = erase_reverse_park_speed_tune();
    if (err == ESP_OK) {
        s_reverse_park_speed_tune_saved = false;
    }
    xSemaphoreGive(s_mutex);
    return err;
}

void chassis_action_tune_print(void)
{
    if (s_mutex == NULL) {
        printf("Chassis tuning: unavailable\n");
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    printf("Chassis PID target overrides:\n");
    bool any_override = false;
    for (int action = CHASSIS_ACTION_MOVE_FORWARD; action < CHASSIS_ACTION_COUNT; ++action) {
        if (s_speed_tunes[action].enabled) {
            printf("  %-24s left=%d right=%d%s\n", action_name((chassis_action_id_t)action),
                   s_speed_tunes[action].left_speed, s_speed_tunes[action].right_speed,
                   action == CHASSIS_ACTION_REVERSE_PARK && s_reverse_park_speed_tune_saved
                       ? " (saved)" : "");
            any_override = true;
        }
    }
    if (!any_override) {
        printf("  none (firmware defaults active)\n");
    }
    xSemaphoreGive(s_mutex);
}
