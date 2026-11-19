#include "imu_heater.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "imu_lsm6ds3tr.h"

/* V1.0 schematic: XIAO D12 / GPIO39 -> R76 -> Q1 heater low-side switch. */
#define IMU_HEATER_GPIO GPIO_NUM_39
#define IMU_HEATER_PWM_FREQUENCY_HZ 20000
#define IMU_HEATER_PWM_RESOLUTION LEDC_TIMER_10_BIT
#define IMU_HEATER_PWM_MAX ((1U << IMU_HEATER_PWM_RESOLUTION) - 1U)
#define IMU_HEATER_TARGET_MC 40000
#define IMU_HEATER_CONTROL_PERIOD_MS 500
#define IMU_HEATER_MAX_DUTY_PERMILLE 600
#define IMU_HEATER_DUTY_STEP_PERMILLE 50
#define IMU_HEATER_FILTER_SHIFT 2
#define IMU_HEATER_KP_PER_MILLE_PER_C 25
#define IMU_HEATER_KI_PER_MILLE_PER_C_S 2
#define IMU_HEATER_INTEGRAL_LIMIT_MC_S 180000
#define IMU_HEATER_TASK_STACK_SIZE 2048
#define IMU_HEATER_TASK_PRIORITY 3

static const char *TAG = "imu_heater";

static bool s_initialized;
static bool s_filter_ready;
static int32_t s_integral_mc_s;
static volatile int32_t s_temperature_mc;
static volatile int32_t s_filtered_temperature_mc;
static volatile uint32_t s_duty_permille;

static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    return value > maximum ? maximum : value;
}

static void set_heater_duty(uint32_t duty_permille)
{
    uint32_t duty = (duty_permille * IMU_HEATER_PWM_MAX) / 1000U;
    (void)ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6, duty);
    (void)ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6);
    s_duty_permille = duty_permille;
}

static uint32_t update_pi(int32_t temperature_mc)
{
    if (!s_filter_ready) {
        s_filtered_temperature_mc = temperature_mc;
        s_filter_ready = true;
    } else {
        s_filtered_temperature_mc +=
            (temperature_mc - s_filtered_temperature_mc) >> IMU_HEATER_FILTER_SHIFT;
    }

    int32_t error_mc = IMU_HEATER_TARGET_MC - s_filtered_temperature_mc;
    s_integral_mc_s += (error_mc * IMU_HEATER_CONTROL_PERIOD_MS) / 1000;
    s_integral_mc_s = clamp_i32(s_integral_mc_s, -IMU_HEATER_INTEGRAL_LIMIT_MC_S,
                                IMU_HEATER_INTEGRAL_LIMIT_MC_S);
    int32_t duty = (IMU_HEATER_KP_PER_MILLE_PER_C * error_mc) / 1000 +
                   (IMU_HEATER_KI_PER_MILLE_PER_C_S * s_integral_mc_s) / 1000;
    duty = clamp_i32(duty, 0, IMU_HEATER_MAX_DUTY_PERMILLE);

    if ((uint32_t)duty > s_duty_permille + IMU_HEATER_DUTY_STEP_PERMILLE) {
        duty = (int32_t)s_duty_permille + IMU_HEATER_DUTY_STEP_PERMILLE;
    } else if (s_duty_permille > (uint32_t)duty + IMU_HEATER_DUTY_STEP_PERMILLE) {
        duty = (int32_t)s_duty_permille - IMU_HEATER_DUTY_STEP_PERMILLE;
    }
    return (uint32_t)duty;
}

static void imu_heater_task(void *arg)
{
    while (true) {
        if (!imu_is_ready()) {
            set_heater_duty(0);
        } else {
            imu_lsm6ds3tr_sample_t sample;
            esp_err_t err = imu_read_sample(&sample);
            if (err != ESP_OK) {
                set_heater_duty(0);
                ESP_LOGW(TAG, "IMU temperature read failed; heater disabled (%s)",
                         esp_err_to_name(err));
            } else {
                s_temperature_mc = sample.temperature_mC;
                set_heater_duty(update_pi(sample.temperature_mC));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(IMU_HEATER_CONTROL_PERIOD_MS));
    }
}

esp_err_t imu_heater_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }
    if (!imu_is_ready()) {
        return ESP_ERR_INVALID_STATE;
    }

    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = IMU_HEATER_PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_2,
        .freq_hz = IMU_HEATER_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_config);
    if (err != ESP_OK) {
        return err;
    }
    const ledc_channel_config_t channel_config = {
        .gpio_num = IMU_HEATER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_6,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_2,
        .duty = 0,
        .hpoint = 0,
    };
    err = ledc_channel_config(&channel_config);
    if (err != ESP_OK) {
        return err;
    }

    s_initialized = true;
    set_heater_duty(0);
    if (xTaskCreate(imu_heater_task, "imu_heater", IMU_HEATER_TASK_STACK_SIZE, NULL,
                    IMU_HEATER_TASK_PRIORITY, NULL) != pdPASS) {
        set_heater_duty(0);
        s_initialized = false;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "IMU heater started on GPIO%d: target 40.0 C, max duty 60%%",
             IMU_HEATER_GPIO);
    return ESP_OK;
}

void imu_heater_print_status(void)
{
    if (!s_initialized) {
        printf("IMU heater: unavailable\n");
        return;
    }
    printf("IMU heater: temp=%ld.%03ld C, filtered=%ld.%03ld C, target=40.000 C, duty=%lu.%lu%%\n",
           (long)(s_temperature_mc / 1000), (long)(s_temperature_mc % 1000),
           (long)(s_filtered_temperature_mc / 1000),
           (long)(s_filtered_temperature_mc % 1000), (unsigned long)(s_duty_permille / 10),
           (unsigned long)(s_duty_permille % 10));
}
