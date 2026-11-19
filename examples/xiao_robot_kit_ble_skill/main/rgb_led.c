#include "rgb_led.h"

#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>

#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_log.h"

#define WS2813_GPIO GPIO_NUM_40
#define WS2813_RMT_RESOLUTION_HZ 10000000 /* 0.1 us per RMT tick */

static const char *TAG = "rgb_led";

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} ws2813_encoder_t;

static rmt_channel_handle_t ws2813_channel;
static rmt_encoder_handle_t ws2813_encoder;
static uint8_t ws2813_pixels[RGB_LED_PIXEL_COUNT * 3];
static bool initialized;

RMT_ENCODER_FUNC_ATTR
static size_t ws2813_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                            const void *primary_data, size_t data_size,
                            rmt_encode_state_t *ret_state)
{
    ws2813_encoder_t *strip_encoder = __containerof(encoder, ws2813_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    switch (strip_encoder->state) {
    case 0:
        encoded_symbols += strip_encoder->bytes_encoder->encode(
            strip_encoder->bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            strip_encoder->state = 1;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto done;
        }
        /* Fall through and append the WS2813 reset interval. */
        __attribute__((fallthrough));
    case 1:
        encoded_symbols += strip_encoder->copy_encoder->encode(
            strip_encoder->copy_encoder, channel, &strip_encoder->reset_code,
            sizeof(strip_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            state |= RMT_ENCODING_COMPLETE;
            strip_encoder->state = RMT_ENCODING_RESET;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
        }
        break;
    default:
        break;
    }

done:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t ws2813_encoder_delete(rmt_encoder_t *encoder)
{
    ws2813_encoder_t *strip_encoder = __containerof(encoder, ws2813_encoder_t, base);
    rmt_del_encoder(strip_encoder->bytes_encoder);
    rmt_del_encoder(strip_encoder->copy_encoder);
    free(strip_encoder);
    return ESP_OK;
}

RMT_ENCODER_FUNC_ATTR
static esp_err_t ws2813_encoder_reset(rmt_encoder_t *encoder)
{
    ws2813_encoder_t *strip_encoder = __containerof(encoder, ws2813_encoder_t, base);
    rmt_encoder_reset(strip_encoder->bytes_encoder);
    rmt_encoder_reset(strip_encoder->copy_encoder);
    strip_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

static esp_err_t ws2813_new_encoder(rmt_encoder_handle_t *ret_encoder)
{
    ws2813_encoder_t *strip_encoder = rmt_alloc_encoder_mem(sizeof(*strip_encoder));
    if (strip_encoder == NULL) {
        return ESP_ERR_NO_MEM;
    }

    strip_encoder->base.encode = ws2813_encode;
    strip_encoder->base.del = ws2813_encoder_delete;
    strip_encoder->base.reset = ws2813_encoder_reset;

    /* Datasheet limits: T0H=0.22..0.38 us, T1H=0.58..1.6 us,
     * T0L=0.58..1.6 us, T1L=0.22..0.42 us. */
    const rmt_bytes_encoder_config_t bytes_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 3, /* 0.3 us */
            .level1 = 0,
            .duration1 = 9, /* 0.9 us */
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 9, /* 0.9 us */
            .level1 = 0,
            .duration1 = 3, /* 0.3 us */
        },
        .flags.msb_first = 1,
    };
    esp_err_t err = rmt_new_bytes_encoder(&bytes_config, &strip_encoder->bytes_encoder);
    if (err != ESP_OK) {
        free(strip_encoder);
        return err;
    }

    const rmt_copy_encoder_config_t copy_config = {};
    err = rmt_new_copy_encoder(&copy_config, &strip_encoder->copy_encoder);
    if (err != ESP_OK) {
        rmt_del_encoder(strip_encoder->bytes_encoder);
        free(strip_encoder);
        return err;
    }

    /* 300 us low: WS2813 reset requirement is greater than 280 us. */
    strip_encoder->reset_code = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = 1500,
        .level1 = 0,
        .duration1 = 1500,
    };
    *ret_encoder = &strip_encoder->base;
    return ESP_OK;
}

esp_err_t rgb_led_init(void)
{
    if (initialized) {
        return ESP_OK;
    }

    const rmt_tx_channel_config_t channel_config = {
        .gpio_num = WS2813_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = WS2813_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&channel_config, &ws2813_channel), TAG,
                        "create RMT channel failed");
    ESP_RETURN_ON_ERROR(ws2813_new_encoder(&ws2813_encoder), TAG,
                        "create WS2813 encoder failed");
    ESP_RETURN_ON_ERROR(rmt_enable(ws2813_channel), TAG, "enable RMT channel failed");

    initialized = true;
    ESP_LOGI(TAG, "WS2813 strip initialized on GPIO%d (%d pixels)", WS2813_GPIO,
             RGB_LED_PIXEL_COUNT);
    return ESP_OK;
}

esp_err_t rgb_led_set_pixel(uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    if (!initialized || index >= RGB_LED_PIXEL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    /* WS2813 wire order is GRB, MSB first. */
    ws2813_pixels[index * 3] = green;
    ws2813_pixels[index * 3 + 1] = red;
    ws2813_pixels[index * 3 + 2] = blue;
    return ESP_OK;
}

esp_err_t rgb_led_show(void)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    const rmt_transmit_config_t transmit_config = {
        .loop_count = 0,
    };
    ESP_RETURN_ON_ERROR(rmt_transmit(ws2813_channel, ws2813_encoder, ws2813_pixels,
                                     sizeof(ws2813_pixels), &transmit_config),
                        TAG, "RMT transmit failed");
    return rmt_tx_wait_all_done(ws2813_channel, 100);
}

esp_err_t rgb_led_fill(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    for (uint8_t i = 0; i < RGB_LED_PIXEL_COUNT; ++i) {
        rgb_led_set_pixel(i, red, green, blue);
    }
    return rgb_led_show();
}

esp_err_t rgb_led_rainbow(void)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    static const uint8_t colors[][3] = {
        {64, 0, 0},  {64, 20, 0}, {64, 64, 0}, {0, 64, 0}, {0, 32, 32},
        {0, 0, 64},  {24, 0, 64}, {64, 0, 32}, {64, 0, 8}, {32, 32, 32},
    };
    for (uint8_t i = 0; i < RGB_LED_PIXEL_COUNT; ++i) {
        rgb_led_set_pixel(i, colors[i][0], colors[i][1], colors[i][2]);
    }
    return rgb_led_show();
}

esp_err_t rgb_led_off(void)
{
    return rgb_led_fill(0, 0, 0);
}
