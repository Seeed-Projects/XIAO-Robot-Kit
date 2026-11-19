#pragma once

#include <stdbool.h>

#include "esp_err.h"

/*
 * One firmware image serves every board: the advertised BLE name lives in NVS
 * instead of being compiled in, so the `btname set <name>` shell command gives
 * each unit its final identity. A saved name survives power cycles and app
 * reflashes; only `btname reset` or a full flash erase clears it.
 *
 * Until a name is saved the board advertises BLE_MOTOR_DEVICE_NAME_PREFIX plus
 * the last two bytes of its BLE address (e.g. XIAO-Robot-Kit-3C7A), so a batch
 * of freshly flashed boards is already distinguishable in a scanner.
 *
 * The length cap keeps the complete name inside one legacy advertising PDU:
 * 31 bytes total, minus 3 bytes of flags and 2 bytes of name header, leaves 26.
 */
#define BLE_MOTOR_DEVICE_NAME_PREFIX "XIAO-Robot-Kit-"
#define BLE_MOTOR_DEVICE_NAME_MAX_LEN 26

esp_err_t ble_motor_start(void);

/* Name currently advertised: the saved one when present, else the MAC-derived default. */
const char *ble_motor_device_name(void);
bool ble_motor_device_name_is_saved(void);

/* MAC-derived name this board falls back to, i.e. what `btname reset` restores. */
const char *ble_motor_device_name_default(void);

/*
 * Persist a new name and apply it immediately. Accepts 1..26 characters from
 * [A-Za-z0-9._-]; returns ESP_ERR_INVALID_ARG for anything else.
 */
esp_err_t ble_motor_device_name_set(const char *name);

/* Drop the saved name and return to the MAC-derived default. */
esp_err_t ble_motor_device_name_reset(void);
