#pragma once

#include <stdbool.h>

#include "esp_err.h"

esp_err_t dance_init(void);
esp_err_t dance_start(void);
esp_err_t dance_start_infinity(void);
esp_err_t dance_start_echo(void);
esp_err_t dance_start_take_me_hand(void);
esp_err_t dance_start_believer(void);
esp_err_t dance_start_new_believer(void);
esp_err_t dance_start_sway(void);
esp_err_t dance_start_greeting(void);
esp_err_t dance_start_handshake(void);
esp_err_t dance_cancel(void);
bool dance_is_active(void);
void dance_stop(void);
