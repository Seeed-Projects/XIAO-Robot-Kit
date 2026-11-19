#pragma once

#include <stdbool.h>

#include "esp_err.h"

/*
 * Battery voltage sense on D16/GPIO10 (J2). The pack node is divided by
 * R109 (100 kΩ, top) and R74 (30 kΩ, bottom), with C11 filtering, so:
 *   Vbat = Vadc * (R109 + R74) / R74
 */
#define BATTERY_ADC_GPIO_NUM 10

esp_err_t battery_init(void);
bool battery_is_ready(void);

/* Reads the pack voltage. All outputs are optional except battery_millivolts. */
esp_err_t battery_read(int *battery_millivolts, int *adc_millivolts, int *raw_average);

void battery_print_status(void);
