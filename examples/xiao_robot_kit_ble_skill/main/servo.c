#include "servo.h"

#include <stdio.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"

#define SERVO_FREQUENCY_HZ 50
#define SERVO_DUTY_RESOLUTION LEDC_TIMER_14_BIT
#define SERVO1_STARTUP_CLOSED_DEG 0
#define SERVO2_STARTUP_CLOSED_DEG 180
/* 500 us stalls at the installed lower stop; 620 us is safely above the tested 10-degree pulse. */
#define SERVO_MIN_PULSE_US 620
#define SERVO_MAX_PULSE_US 2500

static const char *TAG = "servo";

typedef struct {
    gpio_num_t gpio;
    ledc_channel_t channel;
    uint32_t center_pulse_us;
} servo_config_t;

/*
 * The arm end effectors were installed with the original 1500 us command as their
 * physical center. Expanding a single linear range to 620..2500 us would move the
 * command angle 90 to 1560 us, changing that installed pose. Keep a separately
 * calibratable center for each arm and map either side of it independently: this
 * restores the old 90-degree pose without sacrificing the tested safe endpoints.
 */
static const servo_config_t servos[SERVO_COUNT] = {
    { GPIO_NUM_42, LEDC_CHANNEL_4, 1500 }, /* SERVO1 / D15 -> J1, RIGHT */
    { GPIO_NUM_8, LEDC_CHANNEL_5, 1500 },  /* SERVO2 / D9 -> J5, LEFT */
};

static int servo_angles[SERVO_COUNT]; /* -1 = off */
static bool initialized;

static uint32_t pulse_us_to_duty(uint32_t pulse_us)
{
    const uint32_t period_us = 1000000U / SERVO_FREQUENCY_HZ;
    const uint32_t max_duty = (1U << SERVO_DUTY_RESOLUTION) - 1U;
    return (pulse_us * max_duty) / period_us;
}

static uint32_t servo_angle_to_pulse_us(const servo_config_t *servo, int angle)
{
    const int center_angle = (SERVO_ANGLE_MIN + SERVO_ANGLE_MAX) / 2;
    if (angle <= center_angle) {
        return SERVO_MIN_PULSE_US +
               ((servo->center_pulse_us - SERVO_MIN_PULSE_US) * (uint32_t)angle) /
                   (uint32_t)center_angle;
    }

    return servo->center_pulse_us +
           ((SERVO_MAX_PULSE_US - servo->center_pulse_us) * (uint32_t)(angle - center_angle)) /
               (uint32_t)(SERVO_ANGLE_MAX - center_angle);
}

esp_err_t servo_init(void)
{
    if (initialized) {
        return ESP_OK;
    }

    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_DUTY_RESOLUTION,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = SERVO_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG,
                        "configure servo timer failed");

    for (size_t i = 0; i < SERVO_COUNT; ++i) {
        const ledc_channel_config_t channel_config = {
            .gpio_num = servos[i].gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = servos[i].channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_1,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_config), TAG,
                            "configure servo channel failed");
        servo_angles[i] = -1;
    }

    initialized = true;
    /* Reset directly to close_arms: do not visibly pass through 90 degrees. */
    ESP_RETURN_ON_ERROR(servo_set_angle(1, SERVO1_STARTUP_CLOSED_DEG), TAG,
                        "set right startup closed pose failed");
    ESP_RETURN_ON_ERROR(servo_set_angle(2, SERVO2_STARTUP_CLOSED_DEG), TAG,
                        "set left startup closed pose failed");
    ESP_LOGI(TAG, "Two servos initialized (50 Hz) directly at close_arms pose");
    return ESP_OK;
}

esp_err_t servo_set_angle(int servo_id, int angle)
{
    if (!initialized || servo_id < 1 || servo_id > SERVO_COUNT || angle < SERVO_ANGLE_MIN ||
        angle > SERVO_ANGLE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t index = (size_t)(servo_id - 1);
    if (servo_angles[index] == angle) {
        return ESP_OK;
    }
    uint32_t pulse_us = servo_angle_to_pulse_us(&servos[index], angle);
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, servos[index].channel,
                                      pulse_us_to_duty(pulse_us)),
                        TAG, "set servo duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, servos[index].channel), TAG,
                        "update servo duty failed");
    servo_angles[index] = angle;
    return ESP_OK;
}

esp_err_t servo_get_angle(int servo_id, int *angle)
{
    if (!initialized || servo_id < 1 || servo_id > SERVO_COUNT || angle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int current_angle = servo_angles[(size_t)(servo_id - 1)];
    if (current_angle < 0) {
        return ESP_ERR_INVALID_STATE;
    }
    *angle = current_angle;
    return ESP_OK;
}

esp_err_t servo_off(int servo_id)
{
    if (!initialized || servo_id < 1 || servo_id > SERVO_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t index = (size_t)(servo_id - 1);
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, servos[index].channel, 0), TAG,
                        "clear servo duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, servos[index].channel), TAG,
                        "update servo duty failed");
    servo_angles[index] = -1;
    return ESP_OK;
}

void servo_print_status(void)
{
    if (!initialized) {
        printf("Servos: not initialized\n");
        return;
    }
    static const char *const servo_side[SERVO_COUNT] = { "RIGHT (J1)", "LEFT (J5)" };
    for (size_t i = 0; i < SERVO_COUNT; ++i) {
        if (servo_angles[i] >= 0) {
            printf("  SERVO%u %-11s (GPIO%d): %d deg\n", (unsigned)(i + 1), servo_side[i],
                   servos[i].gpio, servo_angles[i]);
        } else {
            printf("  SERVO%u %-11s (GPIO%d): off\n", (unsigned)(i + 1), servo_side[i],
                   servos[i].gpio);
        }
    }
}
