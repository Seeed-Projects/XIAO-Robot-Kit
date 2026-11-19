#pragma once

#include "esp_err.h"

/*
 * Quadrature encoders for the two drive motors, sampled by the PCNT peripheral
 * in 4x mode. Wiring frozen by the V1.0 schematic:
 *
 *   M1 encoder: A=D3 /GPIO4,  B=D14/GPIO41  (via J8)
 *   M2 encoder: A=D17/GPIO13, B=D10/GPIO9   (via J9)
 *
 * PCNT phase actions are calibrated so that physical forward motion is positive
 * for both wheels.
 */
#define ENCODER_COUNT 2

esp_err_t encoder_init(void);
esp_err_t encoder_get_count(int motor_id, int *count);
esp_err_t encoder_reset(int motor_id);
void encoder_reset_all(void);
void encoder_print_status(void);

/* Diagnostic: print the live logic level on each encoder pin (turn the motor by
 * hand and re-run to see whether A/B actually toggle). */
void encoder_debug_levels(void);
