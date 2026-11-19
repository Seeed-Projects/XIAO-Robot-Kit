#include "motor_control.h"

#include <stdbool.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define PWM_FREQUENCY_HZ 20000
#define PWM_DUTY_RESOLUTION LEDC_TIMER_10_BIT
#define PWM_DUTY_MAX ((1U << PWM_DUTY_RESOLUTION) - 1U)

static const char *TAG = "motor_ctrl";

typedef struct {
    gpio_num_t in1_gpio;
    gpio_num_t in2_gpio;
    ledc_channel_t in1_channel;
    ledc_channel_t in2_channel;
    bool invert_direction;
} motor_config_t;

/*
 * Wiring frozen by the V1.0 schematic. `invert_direction` is a runtime flag so
 * polarity can be calibrated after observing rotation, then frozen.
 */
static motor_config_t motors[MOTOR_COUNT] = {
    { GPIO_NUM_2,  GPIO_NUM_3,  LEDC_CHANNEL_0, LEDC_CHANNEL_1, false }, /* M1 / U14 -> J8 */
    { GPIO_NUM_11, GPIO_NUM_12, LEDC_CHANNEL_2, LEDC_CHANNEL_3, true  }, /* M2 / U16 -> J9 */
};

static int motor_speeds[MOTOR_COUNT];
static bool initialized;

static void set_pwm_duty(ledc_channel_t channel, uint32_t duty)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

static void apply_motor_speed(size_t index, int speed_percent)
{
    const motor_config_t *motor = &motors[index];
    int effective_speed = motor->invert_direction ? -speed_percent : speed_percent;
    uint32_t duty = ((uint32_t)(effective_speed < 0 ? -effective_speed : effective_speed) *
                     PWM_DUTY_MAX) / MOTOR_SPEED_MAX;

    /* GC9112: coast both inputs low before changing direction. */
    set_pwm_duty(motor->in1_channel, 0);
    set_pwm_duty(motor->in2_channel, 0);

    if (effective_speed > 0) {
        set_pwm_duty(motor->in1_channel, duty);
    } else if (effective_speed < 0) {
        set_pwm_duty(motor->in2_channel, duty);
    }
}

static void stop_all_internal(void)
{
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
        apply_motor_speed(i, 0);
        motor_speeds[i] = 0;
    }
}

esp_err_t motor_control_init(void)
{
    if (initialized) {
        return ESP_OK;
    }

    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_DUTY_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_config);
    if (err != ESP_OK) {
        return err;
    }

    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
        const motor_config_t *motor = &motors[i];
        const ledc_channel_config_t in1_config = {
            .gpio_num = motor->in1_gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = motor->in1_channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        const ledc_channel_config_t in2_config = {
            .gpio_num = motor->in2_gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = motor->in2_channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };

        err = ledc_channel_config(&in1_config);
        if (err != ESP_OK) {
            return err;
        }
        err = ledc_channel_config(&in2_config);
        if (err != ESP_OK) {
            return err;
        }
    }

    initialized = true;
    stop_all_internal();
    ESP_LOGI(TAG, "Two GC9112 motor channels initialized at %d Hz PWM", PWM_FREQUENCY_HZ);
    return ESP_OK;
}

esp_err_t motor_control_set(int motor_id, int speed_percent)
{
    if (!initialized || motor_id < 1 || motor_id > MOTOR_COUNT ||
        speed_percent < -MOTOR_SPEED_MAX || speed_percent > MOTOR_SPEED_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t index = (size_t)(motor_id - 1);
    apply_motor_speed(index, speed_percent);
    motor_speeds[index] = speed_percent;
    return ESP_OK;
}

esp_err_t motor_control_drive(int left_speed_percent, int right_speed_percent)
{
    if (!initialized || left_speed_percent < -MOTOR_SPEED_MAX ||
        left_speed_percent > MOTOR_SPEED_MAX || right_speed_percent < -MOTOR_SPEED_MAX ||
        right_speed_percent > MOTOR_SPEED_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    apply_motor_speed(MOTOR_LEFT_INDEX, left_speed_percent);
    motor_speeds[MOTOR_LEFT_INDEX] = left_speed_percent;
    apply_motor_speed(MOTOR_RIGHT_INDEX, right_speed_percent);
    motor_speeds[MOTOR_RIGHT_INDEX] = right_speed_percent;
    return ESP_OK;
}

esp_err_t motor_control_get_speed(int motor_id, int *speed_percent)
{
    if (!initialized || speed_percent == NULL || motor_id < 1 || motor_id > MOTOR_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    *speed_percent = motor_speeds[motor_id - 1];
    return ESP_OK;
}

void motor_control_stop_all(void)
{
    if (!initialized) {
        return;
    }

    stop_all_internal();
}

esp_err_t motor_control_set_inverted(int motor_id, bool inverted)
{
    if (motor_id < 1 || motor_id > MOTOR_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    motors[motor_id - 1].invert_direction = inverted;
    /* Re-apply the current command so the new polarity takes effect at once. */
    if (initialized) {
        int index = motor_id - 1;
        apply_motor_speed((size_t)index, motor_speeds[index]);
    }
    return ESP_OK;
}

bool motor_control_is_inverted(int motor_id)
{
    if (motor_id < 1 || motor_id > MOTOR_COUNT) {
        return false;
    }
    return motors[motor_id - 1].invert_direction;
}

void motor_control_print_status(void)
{
    static const char *const wheel_side[MOTOR_COUNT] = { "LEFT (J8)", "RIGHT (J9)" };
    printf("Motor mapping (GC9112 IN1/IN2):\n");
    for (size_t i = 0; i < MOTOR_COUNT; ++i) {
        printf("  M%u %-11s: IN1=GPIO%d IN2=GPIO%d, polarity=%s, command=%d%%\n",
               (unsigned)(i + 1), wheel_side[i], motors[i].in1_gpio, motors[i].in2_gpio,
               motors[i].invert_direction ? "inverted" : "normal", motor_speeds[i]);
    }
    printf("Safety timeout: disabled; use stop or disconnect BLE\n");
}
