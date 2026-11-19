#include "battery.h"

#include <stdio.h>

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"

#define BATTERY_ADC_SAMPLE_COUNT 16
#define BATTERY_DIVIDER_TOP_OHM 100000   /* R109 */
#define BATTERY_DIVIDER_BOTTOM_OHM 30000 /* R74  */

static const char *TAG = "battery";

static adc_oneshot_unit_handle_t battery_adc;
static adc_cali_handle_t battery_adc_cali;
static adc_channel_t battery_adc_channel;
static bool battery_ready;

esp_err_t battery_init(void)
{
    adc_unit_t unit;
    ESP_RETURN_ON_ERROR(adc_oneshot_io_to_channel(BATTERY_ADC_GPIO_NUM, &unit,
                                                  &battery_adc_channel),
                        TAG, "GPIO%d is not an ADC input", BATTERY_ADC_GPIO_NUM);

    const adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = unit,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_config, &battery_adc), TAG,
                        "create battery ADC unit failed");

    const adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(battery_adc, battery_adc_channel,
                                                   &channel_config),
                        TAG, "configure battery ADC channel failed");

    const adc_cali_curve_fitting_config_t calibration_config = {
        .unit_id = unit,
        .chan = battery_adc_channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(adc_cali_create_scheme_curve_fitting(&calibration_config,
                                                             &battery_adc_cali),
                        TAG, "create battery ADC calibration failed");

    battery_ready = true;
    ESP_LOGI(TAG, "Battery ADC ready on GPIO%d", BATTERY_ADC_GPIO_NUM);
    return ESP_OK;
}

bool battery_is_ready(void)
{
    return battery_ready;
}

esp_err_t battery_read(int *battery_millivolts, int *adc_millivolts, int *raw_average)
{
    if (!battery_ready || battery_millivolts == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int total = 0;
    for (int sample = 0; sample < BATTERY_ADC_SAMPLE_COUNT; ++sample) {
        int raw = 0;
        ESP_RETURN_ON_ERROR(adc_oneshot_read(battery_adc, battery_adc_channel, &raw),
                            TAG, "read battery ADC failed");
        total += raw;
    }

    int raw_avg = (total + BATTERY_ADC_SAMPLE_COUNT / 2) / BATTERY_ADC_SAMPLE_COUNT;
    int adc_mv = 0;
    ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(battery_adc_cali, raw_avg, &adc_mv), TAG,
                        "calibrate battery ADC failed");

    *battery_millivolts = (adc_mv * (BATTERY_DIVIDER_TOP_OHM + BATTERY_DIVIDER_BOTTOM_OHM) +
                           BATTERY_DIVIDER_BOTTOM_OHM / 2) /
                          BATTERY_DIVIDER_BOTTOM_OHM;
    if (adc_millivolts != NULL) {
        *adc_millivolts = adc_mv;
    }
    if (raw_average != NULL) {
        *raw_average = raw_avg;
    }
    return ESP_OK;
}

void battery_print_status(void)
{
    if (!battery_ready) {
        printf("Battery: ADC unavailable\n");
        return;
    }

    int battery_mv = 0;
    int adc_mv = 0;
    int raw = 0;
    if (battery_read(&battery_mv, &adc_mv, &raw) != ESP_OK) {
        printf("Battery: read failed\n");
        return;
    }
    printf("Battery: %d.%03d V (%d mV); ADC GPIO%d raw=%d %d mV\n", battery_mv / 1000,
           battery_mv % 1000, battery_mv, BATTERY_ADC_GPIO_NUM, raw, adc_mv);
}
