#include "vofa_telemetry.h"

#include <stdbool.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"

#define VOFA_UART_PORT UART_NUM_0
#define VOFA_UART_TX_GPIO GPIO_NUM_43 /* J10 pin 1: D6 / UART0_TX */
#define VOFA_UART_RX_GPIO GPIO_NUM_44 /* J10 pin 2: D7 / UART0_RX */
#define VOFA_UART_BAUD_RATE 115200
#define VOFA_FRAME_MAX_LEN 192

static const char *TAG = "vofa";
static bool initialized;

esp_err_t vofa_telemetry_init(void)
{
    if (initialized) {
        return ESP_OK;
    }

    const uart_config_t config = {
        .baud_rate = VOFA_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_param_config(VOFA_UART_PORT, &config), TAG,
                        "configure UART0 failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(VOFA_UART_PORT, VOFA_UART_TX_GPIO,
                                     VOFA_UART_RX_GPIO, UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE), TAG,
                        "route UART0 to J10 failed");
    ESP_RETURN_ON_ERROR(uart_driver_install(VOFA_UART_PORT, 256, 512, 0, NULL, 0), TAG,
                        "install UART0 driver failed");

    initialized = true;
    ESP_LOGI(TAG, "VOFA+ FireWater telemetry on J10 UART0: 115200-8N1");
    return ESP_OK;
}

void vofa_telemetry_write(int set_left_speed, int set_right_speed,
                          int pid_left_target_cps, int pid_right_target_cps,
                          int left_encoder_count, int right_encoder_count,
                          int left_cps, int right_cps,
                          int left_pwm_percent, int right_pwm_percent)
{
    if (!initialized) {
        return;
    }

    char frame[VOFA_FRAME_MAX_LEN];
    int length = snprintf(frame, sizeof(frame),
                          "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                          set_left_speed, set_right_speed,
                          pid_left_target_cps, pid_right_target_cps,
                          left_encoder_count, right_encoder_count,
                          left_cps, right_cps, left_pwm_percent, right_pwm_percent);
    if (length > 0 && length < sizeof(frame)) {
        uart_write_bytes(VOFA_UART_PORT, frame, length);
    }
}
