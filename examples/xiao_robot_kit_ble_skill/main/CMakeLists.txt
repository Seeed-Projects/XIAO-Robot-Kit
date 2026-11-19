idf_component_register(SRCS "main.c" "arm_action.c" "chassis_action.c" "dance.c" "motor_control.c" "speed_control.c" "cps_log.c" "vofa_telemetry.c" "ble_motor.c"
                            "encoder.c" "battery.c" "servo.c" "rgb_led.c" "imu_lsm6ds3tr.c" "imu_heater.c" "imu_spin_log.c"
                    INCLUDE_DIRS "."
                    REQUIRES bt console esp_adc esp_app_format esp_driver_gpio
                             esp_driver_i2c esp_driver_ledc esp_driver_pcnt esp_driver_rmt esp_driver_uart
                             esp_timer nvs_flash spiffs)
