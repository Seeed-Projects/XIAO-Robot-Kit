#include "ble_motor.h"

#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_console.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "arm_action.h"
#include "chassis_action.h"
#include "dance.h"
#include "motor_control.h"
#include "speed_control.h"

#define BLE_COMMAND_MAX_LEN 64
#define BLE_STATUS_MAX_LEN 48

/* Same magic+version blob convention the chassis tuning uses (chassis_action.c). */
#define BLE_MOTOR_NAME_NVS_NAMESPACE "ble_motor"
#define BLE_MOTOR_NAME_NVS_KEY "device_name"
#define BLE_MOTOR_NAME_MAGIC 0x424E414DU /* "BNAM" */
#define BLE_MOTOR_NAME_VERSION 1U

typedef struct {
    uint32_t magic;
    uint32_t version;
    char name[BLE_MOTOR_DEVICE_NAME_MAX_LEN + 1];
} saved_device_name_t;

static const char *TAG = "ble_motor";

/* Service UUID: a2f39c10-7e4b-4d21-8a61-0f3bc9d15001 */
static const ble_uuid128_t motor_service_uuid = BLE_UUID128_INIT(
    0x01, 0x50, 0xd1, 0xc9, 0x3b, 0x0f, 0x61, 0x8a,
    0x21, 0x4d, 0x4b, 0x7e, 0x10, 0x9c, 0xf3, 0xa2);

/* Command/status UUID: a2f39c11-7e4b-4d21-8a61-0f3bc9d15001 */
static const ble_uuid128_t motor_value_uuid = BLE_UUID128_INIT(
    0x01, 0x50, 0xd1, 0xc9, 0x3b, 0x0f, 0x61, 0x8a,
    0x21, 0x4d, 0x4b, 0x7e, 0x11, 0x9c, 0xf3, 0xa2);

static char status_value[BLE_STATUS_MAX_LEN] = "READY|$";
static uint16_t status_value_len = sizeof("READY|$") - 1;
static uint16_t motor_value_handle;
static uint8_t own_addr_type;
static char device_name[BLE_MOTOR_DEVICE_NAME_MAX_LEN + 1];
static char default_device_name[BLE_MOTOR_DEVICE_NAME_MAX_LEN + 1];
static bool device_name_saved;

_Static_assert(sizeof(BLE_MOTOR_DEVICE_NAME_PREFIX "XXXX") <= sizeof(device_name),
               "MAC-suffixed default name must fit the advertising-limited buffer");

static int gap_event(struct ble_gap_event *event, void *arg);
static int start_advertising(void);
static esp_err_t apply_device_name(const char *name);

/*
 * Suffix comes from the last two bytes of the BLE address so every board out of
 * the flashing jig already has a distinct name. esp_read_mac(ESP_MAC_BT) matches
 * the address NimBLE advertises; on_sync() re-checks against the address the
 * stack actually reports and corrects the name if they ever diverge.
 */
static void build_default_device_name(void)
{
    uint8_t mac[6] = {0};
    esp_err_t err = esp_read_mac(mac, ESP_MAC_BT);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not read the BT MAC (%s); using suffix 0000",
                 esp_err_to_name(err));
    }
    snprintf(default_device_name, sizeof(default_device_name), "%s%02X%02X",
             BLE_MOTOR_DEVICE_NAME_PREFIX, mac[4], mac[5]);
}

static void ensure_default_device_name(void)
{
    if (default_device_name[0] == '\0') {
        build_default_device_name();
    }
    if (device_name[0] == '\0') {
        strlcpy(device_name, default_device_name, sizeof(device_name));
    }
}

/*
 * Names go into an advertising packet and into `btname set <name>` on the shell,
 * so they stay in the portable subset: no spaces, no quoting, no shell escapes,
 * and readable in any scanner app.
 */
static bool device_name_valid(const char *name)
{
    size_t length = strnlen(name, BLE_MOTOR_DEVICE_NAME_MAX_LEN + 1);
    if (length == 0 || length > BLE_MOTOR_DEVICE_NAME_MAX_LEN) {
        return false;
    }
    for (size_t i = 0; i < length; ++i) {
        char c = name[i];
        bool allowed = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                       (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.';
        if (!allowed) {
            return false;
        }
    }
    return true;
}

static bool saved_device_name_valid(const saved_device_name_t *saved)
{
    return saved->magic == BLE_MOTOR_NAME_MAGIC &&
           saved->version == BLE_MOTOR_NAME_VERSION &&
           saved->name[sizeof(saved->name) - 1] == '\0' &&
           device_name_valid(saved->name);
}

static void load_device_name(void)
{
    ensure_default_device_name();

    nvs_handle_t handle;
    esp_err_t err = nvs_open(BLE_MOTOR_NAME_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved BLE name; advertising as %s (set one with 'btname set <name>')",
                 device_name);
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not open saved BLE name: %s", esp_err_to_name(err));
        return;
    }

    saved_device_name_t saved;
    size_t length = sizeof(saved);
    err = nvs_get_blob(handle, BLE_MOTOR_NAME_NVS_KEY, &saved, &length);
    nvs_close(handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved BLE name; advertising as %s (set one with 'btname set <name>')",
                 device_name);
        return;
    }
    if (err != ESP_OK || length != sizeof(saved) || !saved_device_name_valid(&saved)) {
        ESP_LOGW(TAG, "Saved BLE name is invalid; advertising as %s", device_name);
        return;
    }

    strlcpy(device_name, saved.name, sizeof(device_name));
    device_name_saved = true;
    ESP_LOGI(TAG, "Loaded saved BLE name: %s", device_name);
}

static esp_err_t save_device_name(const char *name)
{
    saved_device_name_t saved = {
        .magic = BLE_MOTOR_NAME_MAGIC,
        .version = BLE_MOTOR_NAME_VERSION,
    };
    strlcpy(saved.name, name, sizeof(saved.name));

    nvs_handle_t handle;
    esp_err_t err = nvs_open(BLE_MOTOR_NAME_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(handle, BLE_MOTOR_NAME_NVS_KEY, &saved, sizeof(saved));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static esp_err_t erase_device_name(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(BLE_MOTOR_NAME_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_erase_key(handle, BLE_MOTOR_NAME_NVS_KEY);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static void set_status(const char *status)
{
    size_t length = strnlen(status, sizeof(status_value) - 1);
    memcpy(status_value, status, length);
    status_value[length] = '\0';
    status_value_len = (uint16_t)length;
}

static void notify_status(void)
{
    if (motor_value_handle == 0) {
        return;
    }

    ble_gatts_chr_updated(motor_value_handle);
}

static bool parse_speed(const char *text, int *speed)
{
    char *end = NULL;
    errno = 0;
    long parsed = strtol(text, &end, 10);

    if (errno == ERANGE || end == text || *end != '\0' ||
        parsed < -MOTOR_SPEED_MAX || parsed > MOTOR_SPEED_MAX ||
        parsed < INT_MIN || parsed > INT_MAX) {
        return false;
    }

    *speed = (int)parsed;
    return true;
}

static void trim_trailing_space(char *text)
{
    size_t length = strlen(text);
    while (length > 0 && (text[length - 1] == '\r' || text[length - 1] == '\n' ||
                          text[length - 1] == ' ' || text[length - 1] == '\t')) {
        text[--length] = '\0';
    }
}

static bool parse_drive_command(char *command, int *left_speed, int *right_speed)
{
    if (strncmp(command, "M|", 2) != 0) {
        return false;
    }

    char *left_text = command + 2;
    char *left_end = strchr(left_text, '|');
    if (left_end == NULL) {
        return false;
    }
    *left_end = '\0';

    char *right_text = left_end + 1;
    char *right_end = strchr(right_text, '|');
    if (right_end == NULL || strcmp(right_end + 1, "$") != 0) {
        return false;
    }
    *right_end = '\0';

    return parse_speed(left_text, left_speed) && parse_speed(right_text, right_speed);
}

/*
 * BLE and the USB shell share ONE command set: any text written to the BLE
 * characteristic is handed to the same console dispatcher the shell uses
 * (esp_console_run). For backward compatibility the frozen YOYO motion frame
 * M|left|right|$ is translated to the equivalent `drive` command first, so the
 * existing RobotPhone/YOYO link keeps working unchanged.
 */
static void execute_command(char *command)
{
    trim_trailing_space(command);

    char probe[BLE_COMMAND_MAX_LEN];
    strlcpy(probe, command, sizeof(probe));

    int left_speed;
    int right_speed;
    bool legacy_drive = parse_drive_command(probe, &left_speed, &right_speed);

    char console_command[BLE_COMMAND_MAX_LEN];
    if (legacy_drive) {
        snprintf(console_command, sizeof(console_command), "drive %d %d",
                 left_speed, right_speed);
    } else {
        strlcpy(console_command, command, sizeof(console_command));
    }

    int command_ret = 0;
    esp_err_t err = esp_console_run(console_command, &command_ret);
    if (err == ESP_ERR_NOT_FOUND) {
        set_status("ERR|UNKNOWN|$");
        ESP_LOGW(TAG, "Unknown BLE command: %s", command);
        return;
    }
    if (err != ESP_OK || command_ret != 0) {
        set_status("ERR|EXEC|$");
        ESP_LOGW(TAG, "BLE command failed (%s, ret=%d): %s",
                 esp_err_to_name(err), command_ret, command);
        return;
    }

    if (legacy_drive) {
        snprintf(status_value, sizeof(status_value), "OK|M|%d|%d|$",
                 left_speed, right_speed);
    } else {
        snprintf(status_value, sizeof(status_value), "OK|%s|$", command);
    }
    status_value_len = (uint16_t)strlen(status_value);
    ESP_LOGI(TAG, "BLE command OK: %s", command);
}

static int gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (attr_handle != motor_value_handle) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        int rc = os_mbuf_append(ctxt->om, status_value, status_value_len);
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t command_len = OS_MBUF_PKTLEN(ctxt->om);
        if (command_len >= BLE_COMMAND_MAX_LEN) {
            set_status("ERR|LENGTH|$");
            notify_status();
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        char command[BLE_COMMAND_MAX_LEN] = {0};
        uint16_t copied = 0;
        int rc = ble_hs_mbuf_to_flat(ctxt->om, command, command_len, &copied);
        if (rc != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }

        command[copied] = '\0';
        ESP_LOGI(TAG, "BLE RX (conn=%d, len=%d): %s",
                 (int)conn_handle, (int)copied, command);
        execute_command(command);
        notify_status();
        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def motor_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &motor_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &motor_value_uuid.u,
                .access_cb = gatt_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                         BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &motor_value_handle,
            },
            { 0 },
        },
    },
    { 0 },
};

static int start_advertising(void)
{
    const char *name = ble_svc_gap_device_name();
    struct ble_hs_adv_fields advertisement = {
        .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,
        .name = (uint8_t *)name,
        .name_len = strlen(name),
        .name_is_complete = 1,
    };
    int rc = ble_gap_adv_set_fields(&advertisement);
    if (rc != 0) {
        ESP_LOGE(TAG, "Unable to set advertising data: rc=%d", rc);
        return rc;
    }

    struct ble_hs_adv_fields scan_response = {
        .uuids128 = &motor_service_uuid,
        .num_uuids128 = 1,
        .uuids128_is_complete = 1,
    };
    rc = ble_gap_adv_rsp_set_fields(&scan_response);
    if (rc != 0) {
        ESP_LOGE(TAG, "Unable to set scan response: rc=%d", rc);
        return rc;
    }

    struct ble_gap_adv_params parameters = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &parameters, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Unable to start advertising: rc=%d", rc);
        return rc;
    }

    ESP_LOGI(TAG, "Advertising as %s (general discoverable, connectable)", name);
    return 0;
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            set_status("CONNECTED|$");
            ESP_LOGI(TAG, "Central connected (handle=%u)", event->connect.conn_handle);
        } else {
            ESP_LOGW(TAG, "Connection attempt failed: status=%d", event->connect.status);
            start_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        (void)chassis_action_cancel();
        (void)dance_cancel();
        speed_control_stop();
        (void)arm_action_cancel();
        set_status("DISCONNECTED|$");
        ESP_LOGW(TAG, "Central disconnected (reason=%d); motors stopped",
                 event->disconnect.reason);
        start_advertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "Notifications %s for handle=%u",
                 event->subscribe.cur_notify ? "enabled" : "disabled",
                 event->subscribe.attr_handle);
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated to %u", event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGW(TAG, "Advertising stopped (reason=%d)", event->adv_complete.reason);
        start_advertising();
        return 0;

    default:
        return 0;
    }
}

static void on_reset(int reason)
{
    (void)chassis_action_cancel();
    (void)dance_cancel();
    speed_control_stop();
    (void)arm_action_cancel();
    ESP_LOGW(TAG, "NimBLE reset (reason=%d); motors stopped", reason);
}

/*
 * esp_read_mac() returns the address big-endian (mac[0] printed first) while
 * ble_hs_id_copy_addr() returns it little-endian (address[5] printed first), so
 * the two displayed trailing bytes are mac[4],mac[5] and address[1],address[0]
 * respectively. This normally agrees with the name already built from the BT
 * MAC; it exists so the advertised name can never disagree with the advertised
 * address if that assumption ever breaks. A saved name always wins.
 */
static void correct_default_device_name(const uint8_t *address)
{
    char corrected[sizeof(default_device_name)];
    snprintf(corrected, sizeof(corrected), "%s%02X%02X",
             BLE_MOTOR_DEVICE_NAME_PREFIX, address[1], address[0]);

    ensure_default_device_name();
    if (strcmp(corrected, default_device_name) == 0) {
        return;
    }

    ESP_LOGW(TAG, "Default name %s did not match the BLE address; using %s",
             default_device_name, corrected);
    strlcpy(default_device_name, corrected, sizeof(default_device_name));
    if (device_name_saved) {
        return;
    }

    strlcpy(device_name, corrected, sizeof(device_name));
    (void)apply_device_name(device_name);
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "No usable BLE address: rc=%d", rc);
        return;
    }

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Unable to infer BLE address type: rc=%d", rc);
        return;
    }

    uint8_t address[6];
    rc = ble_hs_id_copy_addr(own_addr_type, address, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "BLE address: %02x:%02x:%02x:%02x:%02x:%02x",
                 address[5], address[4], address[3], address[2], address[1], address[0]);
        correct_default_device_name(address);
    }

    /* The name is what operators read off the monitor while flashing a batch. */
    ESP_LOGI(TAG, "BLE device name: %s (%s)", ble_motor_device_name(),
             device_name_saved ? "saved in NVS" : "default from MAC");

    set_status("READY|$");
    start_advertising();
}

static void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/*
 * The advertising payload is built once when advertising starts, so a rename
 * only reaches scanners after a restart of the advertisement. While a central is
 * connected there is no advertisement to restart; gap_event() starts a fresh one
 * carrying the new name on disconnect.
 */
static void restart_advertising_for_new_name(void)
{
    if (!ble_gap_adv_active()) {
        return;
    }
    int rc = ble_gap_adv_stop();
    if (rc != 0) {
        ESP_LOGW(TAG, "Could not stop advertising to apply the new name: rc=%d", rc);
        return;
    }
    start_advertising();
}

const char *ble_motor_device_name(void)
{
    ensure_default_device_name();
    return device_name;
}

const char *ble_motor_device_name_default(void)
{
    ensure_default_device_name();
    return default_device_name;
}

bool ble_motor_device_name_is_saved(void)
{
    return device_name_saved;
}

static esp_err_t apply_device_name(const char *name)
{
    int rc = ble_svc_gap_device_name_set(name);
    if (rc != 0) {
        ESP_LOGE(TAG, "Unable to set device name: rc=%d", rc);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ble_motor_device_name_set(const char *name)
{
    if (name == NULL || !device_name_valid(name)) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Apply before persisting so NVS never holds a name the stack rejected. */
    esp_err_t err = apply_device_name(name);
    if (err != ESP_OK) {
        (void)apply_device_name(device_name);
        return err;
    }

    err = save_device_name(name);
    if (err != ESP_OK) {
        (void)apply_device_name(device_name);
        return err;
    }

    strlcpy(device_name, name, sizeof(device_name));
    device_name_saved = true;
    restart_advertising_for_new_name();
    ESP_LOGI(TAG, "BLE name saved: %s", device_name);
    return ESP_OK;
}

esp_err_t ble_motor_device_name_reset(void)
{
    ensure_default_device_name();

    esp_err_t err = apply_device_name(default_device_name);
    if (err != ESP_OK) {
        (void)apply_device_name(device_name);
        return err;
    }

    err = erase_device_name();
    if (err != ESP_OK) {
        (void)apply_device_name(device_name);
        return err;
    }

    strlcpy(device_name, default_device_name, sizeof(device_name));
    device_name_saved = false;
    restart_advertising_for_new_name();
    ESP_LOGI(TAG, "BLE name reset to the default: %s", device_name);
    return ESP_OK;
}

esp_err_t ble_motor_start(void)
{
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        return err;
    }

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(motor_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Unable to count GATT configuration: rc=%d", rc);
        return ESP_FAIL;
    }
    rc = ble_gatts_add_svcs(motor_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Unable to add GATT service: rc=%d", rc);
        return ESP_FAIL;
    }
    load_device_name();
    err = apply_device_name(device_name);
    if (err != ESP_OK) {
        return err;
    }

    nimble_port_freertos_init(nimble_host_task);
    return ESP_OK;
}
