#pragma once

#include <stdint.h>

#include "esp_err.h"

/*
 * Addressable RGB strip (WS2813) on D14/GPIO41, external 10-pixel strip on J7.
 * The MCU drives the data line at 3.3 V; Q23 (LBSS138) level-shifts it to the
 * strip's 5 V domain and is non-inverting, so standard WS2813 timing applies.
 */
#define RGB_LED_PIXEL_COUNT 10

esp_err_t rgb_led_init(void);
esp_err_t rgb_led_set_pixel(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);
esp_err_t rgb_led_fill(uint8_t red, uint8_t green, uint8_t blue);
esp_err_t rgb_led_show(void);
esp_err_t rgb_led_rainbow(void);
esp_err_t rgb_led_off(void);
