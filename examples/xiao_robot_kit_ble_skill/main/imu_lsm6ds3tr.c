#include "imu_lsm6ds3tr.h"

#include <stdio.h>

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

/*
 * Hardware evidence, Honor Robot Phone-Maker V0.9 schematic:
 *
 *   U18 LSM6DS3TR-C SDA (pin 14) -> D4/I2C1_SDA -> XIAO D4 / GPIO5
 *   U18 LSM6DS3TR-C SCL (pin 13) -> D5/I2C1_SCL -> XIAO D5 / GPIO6
 *   U18 SDO/DA0 is grounded, selecting 7-bit I2C address 0x6A.
 *   U18 CS is pulled up to IMU_VDD, selecting I2C rather than SPI.
 *
 * INT1 is only pulled up through R112 in this board revision; it is not
 * routed to an ESP32 GPIO.  Therefore this driver uses polling and does not
 * enable an unobservable data-ready interrupt.
 */
#define IMU_I2C_PORT I2C_NUM_0
#define IMU_I2C_SDA_GPIO GPIO_NUM_5
#define IMU_I2C_SCL_GPIO GPIO_NUM_6
#define IMU_I2C_CLOCK_HZ 400000U
#define IMU_I2C_ADDRESS 0x6AU
#define IMU_I2C_TIMEOUT_MS 20

#define LSM6DS3TR_REG_WHO_AM_I 0x0FU
#define LSM6DS3TR_WHO_AM_I_VALUE 0x6AU
#define LSM6DS3TR_REG_CTRL1_XL 0x10U
#define LSM6DS3TR_REG_CTRL2_G 0x11U
#define LSM6DS3TR_REG_CTRL3_C 0x12U
#define LSM6DS3TR_REG_OUT_TEMP_L 0x20U

/* 104 Hz, +/-2 g for a finer stationary gravity-direction measurement. */
#define LSM6DS3TR_CTRL1_XL_104HZ_2G 0x40U
#define LSM6DS3TR_CTRL2_G_104HZ_500DPS 0x44U
#define LSM6DS3TR_CTRL3_BDU_AUTO_INCREMENT 0x44U

static const char *TAG = "imu";

static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_imu_device;
static bool s_initialized;
static uint8_t s_who_am_i;
static esp_err_t s_last_error = ESP_ERR_INVALID_STATE;

static int16_t decode_le16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void release_i2c(void)
{
    if (s_imu_device != NULL) {
        (void)i2c_master_bus_rm_device(s_imu_device);
        s_imu_device = NULL;
    }
    if (s_i2c_bus != NULL) {
        (void)i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
    }
    s_initialized = false;
}

static esp_err_t read_registers(uint8_t reg, uint8_t *data, size_t length)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = i2c_master_transmit_receive(s_imu_device, &reg, sizeof(reg), data, length,
                                                IMU_I2C_TIMEOUT_MS);
    s_last_error = err;
    return err;
}

static esp_err_t write_register(uint8_t reg, uint8_t value)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint8_t data[] = {reg, value};
    esp_err_t err = i2c_master_transmit(s_imu_device, data, sizeof(data), IMU_I2C_TIMEOUT_MS);
    s_last_error = err;
    return err;
}

static esp_err_t configure_sensor(void)
{
    uint8_t ctrl3_c;
    esp_err_t err = read_registers(LSM6DS3TR_REG_CTRL3_C, &ctrl3_c, sizeof(ctrl3_c));
    if (err != ESP_OK) {
        return err;
    }

    ctrl3_c |= LSM6DS3TR_CTRL3_BDU_AUTO_INCREMENT;
    err = write_register(LSM6DS3TR_REG_CTRL3_C, ctrl3_c);
    if (err != ESP_OK) {
        return err;
    }
    err = write_register(LSM6DS3TR_REG_CTRL1_XL, LSM6DS3TR_CTRL1_XL_104HZ_2G);
    if (err != ESP_OK) {
        return err;
    }
    return write_register(LSM6DS3TR_REG_CTRL2_G, LSM6DS3TR_CTRL2_G_104HZ_500DPS);
}

esp_err_t imu_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    release_i2c();

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = IMU_I2C_PORT,
        .sda_io_num = IMU_I2C_SDA_GPIO,
        .scl_io_num = IMU_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = false, /* R72/R100 and R98/R99 are fitted externally. */
        .flags.allow_pd = false,
    };
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_I2C_ADDRESS,
        .scl_speed_hz = IMU_I2C_CLOCK_HZ,
        .scl_wait_us = 0,
        .flags.disable_ack_check = false,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &s_i2c_bus);
    if (err != ESP_OK) {
        goto fail;
    }
    err = i2c_master_probe(s_i2c_bus, IMU_I2C_ADDRESS, IMU_I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        goto fail;
    }
    err = i2c_master_bus_add_device(s_i2c_bus, &device_config, &s_imu_device);
    if (err != ESP_OK) {
        goto fail;
    }

    /* The transport is usable after the device handle exists; enable register access for setup. */
    s_initialized = true;
    err = read_registers(LSM6DS3TR_REG_WHO_AM_I, &s_who_am_i, sizeof(s_who_am_i));
    if (err != ESP_OK) {
        goto fail;
    }
    if (s_who_am_i != LSM6DS3TR_WHO_AM_I_VALUE) {
        err = ESP_ERR_INVALID_RESPONSE;
        goto fail;
    }
    err = configure_sensor();
    if (err != ESP_OK) {
        goto fail;
    }

    s_last_error = ESP_OK;
    ESP_LOGI(TAG, "LSM6DS3TR-C ready: I2C%d SDA=GPIO%d SCL=GPIO%d addr=0x%02X, 104 Hz",
             IMU_I2C_PORT, IMU_I2C_SDA_GPIO, IMU_I2C_SCL_GPIO, IMU_I2C_ADDRESS);
    return ESP_OK;

fail:
    s_last_error = err;
    ESP_LOGW(TAG, "LSM6DS3TR-C initialization failed: %s", esp_err_to_name(err));
    release_i2c();
    return err;
}

esp_err_t imu_reinit(void)
{
    release_i2c();
    return imu_init();
}

bool imu_is_ready(void)
{
    return s_initialized;
}

esp_err_t imu_read_register(uint8_t reg, uint8_t *value)
{
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return read_registers(reg, value, 1);
}

esp_err_t imu_read_sample(imu_lsm6ds3tr_sample_t *sample)
{
    if (sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * CTRL3_C.IF_INC is set at initialization.  Reading 0x20..0x2D in one
     * transaction keeps temperature, gyro, and accelerometer near one sample
     * instant and avoids an inconsistent six-axis shell read.
     */
    uint8_t data[14];
    esp_err_t err = read_registers(LSM6DS3TR_REG_OUT_TEMP_L, data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }

    int16_t raw_temperature = decode_le16(&data[0]);
    sample->temperature_mC = 25000 + ((int32_t)raw_temperature * 1000) / 256;
    sample->gyro_x = decode_le16(&data[2]);
    sample->gyro_y = decode_le16(&data[4]);
    sample->gyro_z = decode_le16(&data[6]);
    sample->accel_x = decode_le16(&data[8]);
    sample->accel_y = decode_le16(&data[10]);
    sample->accel_z = decode_le16(&data[12]);
    return ESP_OK;
}

void imu_print_status(void)
{
    if (!s_initialized) {
        printf("IMU: unavailable (last error: %s)\n", esp_err_to_name(s_last_error));
        printf("  expected LSM6DS3TR-C at I2C0, GPIO5(SDA)/GPIO6(SCL), address 0x6A\n");
        return;
    }

    printf("IMU: LSM6DS3TR-C ready, WHO_AM_I=0x%02X\n", s_who_am_i);
    printf("  I2C0 GPIO5(SDA) GPIO6(SCL), address 0x6A, 400 kHz\n");
    printf("  accel: 104 Hz, +/-2 g; gyro: 104 Hz, +/-500 dps\n");
    printf("  INT1 is not routed to ESP32 on this board revision; polling is active\n");
}
