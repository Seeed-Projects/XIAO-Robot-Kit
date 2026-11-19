#include "encoder.h"

#include <stdio.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_check.h"
#include "esp_log.h"

#define ENCODER_LIMIT 30000
#define ENCODER_GLITCH_FILTER_NS 1000

static const char *TAG = "encoder";

typedef struct {
    gpio_num_t phase_a_gpio;
    gpio_num_t phase_b_gpio;
    bool reverse_pcnt_direction;
} encoder_config_t;

static const encoder_config_t encoders[ENCODER_COUNT] = {
    /* V0.9 hardware calibration: physical forward motion increments both counts. */
    { GPIO_NUM_4,  GPIO_NUM_41, true  }, /* M1: A=D3/GPIO4,  B=D14/GPIO41 */
    { GPIO_NUM_13, GPIO_NUM_9,  false }, /* M2: A=D17/GPIO13, B=D10/GPIO9 */
};

static pcnt_unit_handle_t encoder_units[ENCODER_COUNT];
static bool initialized;

static esp_err_t configure_encoder_inputs(const encoder_config_t *encoder)
{
    const gpio_config_t input_config = {
        .pin_bit_mask = (1ULL << encoder->phase_a_gpio) | (1ULL << encoder->phase_b_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&input_config);
}

static esp_err_t init_one_encoder(size_t index, pcnt_unit_handle_t *unit_out)
{
    const encoder_config_t *encoder = &encoders[index];
    const pcnt_channel_edge_action_t a_rising_action = encoder->reverse_pcnt_direction ?
        PCNT_CHANNEL_EDGE_ACTION_INCREASE : PCNT_CHANNEL_EDGE_ACTION_DECREASE;
    const pcnt_channel_edge_action_t a_falling_action = encoder->reverse_pcnt_direction ?
        PCNT_CHANNEL_EDGE_ACTION_DECREASE : PCNT_CHANNEL_EDGE_ACTION_INCREASE;
    const pcnt_channel_edge_action_t b_rising_action = encoder->reverse_pcnt_direction ?
        PCNT_CHANNEL_EDGE_ACTION_DECREASE : PCNT_CHANNEL_EDGE_ACTION_INCREASE;
    const pcnt_channel_edge_action_t b_falling_action = encoder->reverse_pcnt_direction ?
        PCNT_CHANNEL_EDGE_ACTION_INCREASE : PCNT_CHANNEL_EDGE_ACTION_DECREASE;

    ESP_RETURN_ON_ERROR(configure_encoder_inputs(encoder), TAG,
                        "configure encoder GPIO inputs failed");

    const pcnt_unit_config_t unit_config = {
        .low_limit = -ENCODER_LIMIT,
        .high_limit = ENCODER_LIMIT,
        .flags.accum_count = true,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_config, unit_out), TAG,
                        "create PCNT unit failed");

    const pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = ENCODER_GLITCH_FILTER_NS,
    };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(*unit_out, &filter_config), TAG,
                        "configure PCNT filter failed");

    /* 4x quadrature: A edges counted, steered by B level; B edges counted, steered
     * by A level. Direction (sign) depends on phase order vs rotation. */
    pcnt_channel_handle_t channel_a;
    const pcnt_chan_config_t a_config = {
        .edge_gpio_num = encoder->phase_a_gpio,
        .level_gpio_num = encoder->phase_b_gpio,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(*unit_out, &a_config, &channel_a), TAG,
                        "create PCNT A channel failed");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(channel_a,
                                                      a_rising_action, a_falling_action),
                        TAG, "configure PCNT A edges failed");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(channel_a,
                                                       PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                       PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
                        TAG, "configure PCNT A level failed");

    pcnt_channel_handle_t channel_b;
    const pcnt_chan_config_t b_config = {
        .edge_gpio_num = encoder->phase_b_gpio,
        .level_gpio_num = encoder->phase_a_gpio,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(*unit_out, &b_config, &channel_b), TAG,
                        "create PCNT B channel failed");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(channel_b,
                                                      b_rising_action, b_falling_action),
                        TAG, "configure PCNT B edges failed");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(channel_b,
                                                       PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                       PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
                        TAG, "configure PCNT B level failed");

    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(*unit_out, -ENCODER_LIMIT), TAG,
                        "add PCNT low watch point failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(*unit_out, ENCODER_LIMIT), TAG,
                        "add PCNT high watch point failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(*unit_out), TAG, "enable PCNT failed");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(*unit_out), TAG, "clear PCNT failed");
    return pcnt_unit_start(*unit_out);
}

esp_err_t encoder_init(void)
{
    if (initialized) {
        return ESP_OK;
    }

    for (size_t i = 0; i < ENCODER_COUNT; ++i) {
        esp_err_t err = init_one_encoder(i, &encoder_units[i]);
        if (err != ESP_OK) {
            return err;
        }
    }

    initialized = true;
    ESP_LOGI(TAG, "Two quadrature encoders initialized (4x PCNT)");
    return ESP_OK;
}

esp_err_t encoder_get_count(int motor_id, int *count)
{
    if (!initialized || count == NULL || motor_id < 1 || motor_id > ENCODER_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    return pcnt_unit_get_count(encoder_units[motor_id - 1], count);
}

esp_err_t encoder_reset(int motor_id)
{
    if (!initialized || motor_id < 1 || motor_id > ENCODER_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    return pcnt_unit_clear_count(encoder_units[motor_id - 1]);
}

void encoder_reset_all(void)
{
    if (!initialized) {
        return;
    }
    for (size_t i = 0; i < ENCODER_COUNT; ++i) {
        ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder_units[i]));
    }
}

void encoder_debug_levels(void)
{
    if (!initialized) {
        printf("Encoders: not initialized\n");
        return;
    }
    static const char *const wheel_side[ENCODER_COUNT] = { "RIGHT", "LEFT" };
    for (size_t i = 0; i < ENCODER_COUNT; ++i) {
        printf("  M%u %-5s: A(GPIO%d)=%d  B(GPIO%d)=%d\n", (unsigned)(i + 1),
               wheel_side[i], encoders[i].phase_a_gpio,
               gpio_get_level(encoders[i].phase_a_gpio), encoders[i].phase_b_gpio,
               gpio_get_level(encoders[i].phase_b_gpio));
    }
}

void encoder_print_status(void)
{
    if (!initialized) {
        printf("Encoders: not initialized\n");
        return;
    }
    static const char *const wheel_side[ENCODER_COUNT] = { "RIGHT", "LEFT" };
    for (size_t i = 0; i < ENCODER_COUNT; ++i) {
        int count = 0;
        ESP_ERROR_CHECK(pcnt_unit_get_count(encoder_units[i], &count));
        printf("  M%u %-5s encoder (A=GPIO%d B=GPIO%d, pull-up): count=%d\n",
               (unsigned)(i + 1), wheel_side[i], encoders[i].phase_a_gpio,
               encoders[i].phase_b_gpio, count);
    }
}
