#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "battery.h"
#include "arm_action.h"
#include "ble_motor.h"
#include "chassis_action.h"
#include "cps_log.h"
#include "dance.h"
#include "encoder.h"
#include "imu_heater.h"
#include "imu_lsm6ds3tr.h"
#include "imu_spin_log.h"
#include "motor_control.h"
#include "rgb_led.h"
#include "servo.h"
#include "speed_control.h"

#define DEFAULT_SPEED_PERCENT 40

static const char *TAG = "robot_chassis";

static bool parse_int_range(const char *text, int minimum, int maximum, int *value)
{
    char *end = NULL;
    errno = 0;
    long parsed = strtol(text, &end, 10);

    if (errno == ERANGE || end == text || *end != '\0' ||
        parsed < minimum || parsed > maximum ||
        parsed < INT_MIN || parsed > INT_MAX) {
        return false;
    }

    *value = (int)parsed;
    return true;
}

static int parse_optional_speed(int argc, char **argv, int *speed)
{
    if (argc == 1) {
        *speed = DEFAULT_SPEED_PERCENT;
        return 0;
    }
    if (argc != 2 || !parse_int_range(argv[1], 1, MOTOR_SPEED_MAX, speed)) {
        return 1;
    }
    return 0;
}

static int cancel_chassis_action_for_manual_control(void)
{
    esp_err_t dance_err = dance_cancel();
    if (dance_err != ESP_OK) {
        printf("Could not cancel dance: %s\n", esp_err_to_name(dance_err));
        return 1;
    }
    esp_err_t err = chassis_action_cancel();
    if (err != ESP_OK) {
        printf("Could not cancel chassis action: %s\n", esp_err_to_name(err));
        return 1;
    }
    return 0;
}

static int cmd_motor(int argc, char **argv)
{
    int motor_id;
    int speed;
    if (argc != 3 ||
        !parse_int_range(argv[1], 1, MOTOR_COUNT, &motor_id) ||
        !parse_int_range(argv[2], -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX, &speed)) {
        printf("Usage: motor <1-%d> <-100..100>\n", MOTOR_COUNT);
        return 1;
    }

    if (cancel_chassis_action_for_manual_control() != 0) {
        return 1;
    }
    speed_control_stop();
    ESP_ERROR_CHECK(motor_control_set(motor_id, speed));
    printf("Motor M%d set to %d%% (continues until stop)\n", motor_id, speed);
    return 0;
}

static int cmd_motorinv(int argc, char **argv)
{
    int motor_id;
    int inverted;
    if (argc != 3 ||
        !parse_int_range(argv[1], 1, MOTOR_COUNT, &motor_id) ||
        !parse_int_range(argv[2], 0, 1, &inverted)) {
        printf("Usage: motorinv <1-%d> <0|1>  (calibrate motor polarity)\n", MOTOR_COUNT);
        return 1;
    }

    if (cancel_chassis_action_for_manual_control() != 0) {
        return 1;
    }
    speed_control_stop();
    ESP_ERROR_CHECK(motor_control_set_inverted(motor_id, inverted != 0));
    printf("Motor M%d polarity %s\n", motor_id, inverted ? "inverted" : "normal");
    return 0;
}

static int cmd_drive(int argc, char **argv)
{
    int left_speed;
    int right_speed;
    if (argc != 3 ||
        !parse_int_range(argv[1], -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX, &left_speed) ||
        !parse_int_range(argv[2], -MOTOR_SPEED_MAX, MOTOR_SPEED_MAX, &right_speed)) {
        printf("Usage: drive <left -100..100> <right -100..100>\n");
        return 1;
    }

    if (cancel_chassis_action_for_manual_control() != 0) {
        return 1;
    }
    speed_control_stop();
    ESP_ERROR_CHECK(motor_control_drive(left_speed, right_speed));
    printf("Left=%d%% Right=%d%% (continues until stop)\n", left_speed, right_speed);
    return 0;
}

typedef enum {
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_LEFT,
    MOVE_RIGHT,
} move_direction_t;

static int cmd_move(int argc, char **argv, move_direction_t direction)
{
    int speed;
    if (parse_optional_speed(argc, argv, &speed) != 0) {
        printf("Usage: %s [speed 1..100]\n", argv[0]);
        return 1;
    }

    if (cancel_chassis_action_for_manual_control() != 0) {
        return 1;
    }
    speed_control_stop();
    switch (direction) {
    case MOVE_FORWARD:
        ESP_ERROR_CHECK(motor_control_drive(speed, speed));
        break;
    case MOVE_BACKWARD:
        ESP_ERROR_CHECK(motor_control_drive(-speed, -speed));
        break;
    case MOVE_LEFT:
        ESP_ERROR_CHECK(motor_control_drive(-speed, speed));
        break;
    case MOVE_RIGHT:
        ESP_ERROR_CHECK(motor_control_drive(speed, -speed));
        break;
    }

    printf("%s at %d%% (continues until stop)\n", argv[0], speed);
    return 0;
}

static int cmd_forward(int argc, char **argv)
{
    return cmd_move(argc, argv, MOVE_FORWARD);
}

static int cmd_backward(int argc, char **argv)
{
    return cmd_move(argc, argv, MOVE_BACKWARD);
}

static int cmd_left(int argc, char **argv)
{
    return cmd_move(argc, argv, MOVE_LEFT);
}

static int cmd_right(int argc, char **argv)
{
    return cmd_move(argc, argv, MOVE_RIGHT);
}

static int cmd_stop(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: stop\n");
        return 1;
    }

    esp_err_t chassis_err = chassis_action_cancel();
    esp_err_t dance_err = dance_cancel();
    /* chassis_action_cancel() sets its cancel flag before stopping PID, so its
     * task cannot resume the controller after this explicit stop. */
    speed_control_stop();
    esp_err_t arm_err = arm_action_cancel();
    if (chassis_err != ESP_OK) {
        printf("Motors stopped, but chassis action cancellation failed: %s\n",
               esp_err_to_name(chassis_err));
        return 1;
    }
    if (dance_err != ESP_OK) {
        printf("Motors stopped, but dance cancellation failed: %s\n", esp_err_to_name(dance_err));
        return 1;
    }
    if (arm_err != ESP_OK) {
        printf("Motors stopped, but arm action cancellation failed: %s\n", esp_err_to_name(arm_err));
        return 1;
    }
    printf("All motors stopped; active chassis, arm and dance actions cancelled\n");
    return 0;
}

static int cmd_speed(int argc, char **argv)
{
    int left_speed;
    int right_speed;
    if (argc != 3 || !parse_int_range(argv[1], -SPEED_CONTROL_SPEED_MAX,
                                      SPEED_CONTROL_SPEED_MAX, &left_speed) ||
        !parse_int_range(argv[2], -SPEED_CONTROL_SPEED_MAX,
                         SPEED_CONTROL_SPEED_MAX, &right_speed)) {
        printf("Usage: speed <left -%d..%d> <right -%d..%d>\n",
               SPEED_CONTROL_SPEED_MAX, SPEED_CONTROL_SPEED_MAX,
               SPEED_CONTROL_SPEED_MAX, SPEED_CONTROL_SPEED_MAX);
        return 1;
    }

    if (cancel_chassis_action_for_manual_control() != 0) {
        return 1;
    }
    ESP_ERROR_CHECK(speed_control_set_target(left_speed, right_speed));
    printf("PID speed setpoint: left=%d right=%d (continues until stop)\n",
           left_speed, right_speed);
    return 0;
}

static int start_chassis_action(chassis_action_id_t action, const char *name)
{
    esp_err_t dance_err = dance_cancel();
    if (dance_err != ESP_OK) {
        printf("Could not cancel dance: %s\n", esp_err_to_name(dance_err));
        return 1;
    }
    esp_err_t err = chassis_action_start(action);
    if (err == ESP_ERR_INVALID_STATE) {
        printf("Another required chassis or arm action is running, or IMU calibration is required; "
               "use stop or imu calibrate\n");
        return 1;
    }
    if (err != ESP_OK) {
        printf("Could not start %s: %s\n", name, esp_err_to_name(err));
        return 1;
    }
    if (action == CHASSIS_ACTION_GREETING) {
        printf("greeting started: right-arm wave + PID setpoint +55/+55 for 3 s, then stop\n");
    } else if (action == CHASSIS_ACTION_MOVE_FORWARD || action == CHASSIS_ACTION_MOVE_BACKWARD) {
        printf("%s started: IMU heading hold with PID base %s55/%s55 for 1 s, then stop\n", name,
               action == CHASSIS_ACTION_MOVE_FORWARD ? "+" : "-",
               action == CHASSIS_ACTION_MOVE_FORWARD ? "+" : "-");
    } else if (action == CHASSIS_ACTION_TURN_LEFT) {
        printf("turn_left started: PID left=0 right=+70 for 1.5 s, then stop\n");
    } else if (action == CHASSIS_ACTION_TURN_RIGHT) {
        printf("turn_right started: PID left=+70 right=0 for 1.5 s, then stop\n");
    } else if (action == CHASSIS_ACTION_REVERSE_PARK) {
        printf("reverse_park started: clockwise -50/-100 fan-shaped reverse to IMU 90 degrees\n");
    } else if (action == CHASSIS_ACTION_SPIN_CLOCKWISE) {
        printf("spin_clockwise started: PID left=+100 right=-100 until IMU reaches one turn\n");
    } else if (action == CHASSIS_ACTION_SPIN_COUNTERCLOCKWISE) {
        printf("spin_counterclockwise started: PID left=-100 right=+100 until IMU reaches one turn\n");
    } else if (action == CHASSIS_ACTION_EMOTION_HAPPY) {
        printf("emotion_happy started: rapid arm raise + two swings and IMU-measured 360 deg spin\n");
    } else if (action == CHASSIS_ACTION_EMOTION_SCARED) {
        printf("emotion_scared started: rapid arm close and PID -80/-80 for 500 ms\n");
    } else {
        printf("emotion_angry started: arm open/close twice and four rapid PID wobble segments\n");
    }
    return 0;
}

static int cmd_move_forward(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: move_forward\n");
        return 1;
    }
    return start_chassis_action(CHASSIS_ACTION_MOVE_FORWARD, "move_forward");
}

static int cmd_move_backward(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: move_backward\n");
        return 1;
    }
    return start_chassis_action(CHASSIS_ACTION_MOVE_BACKWARD, "move_backward");
}

static int cmd_greeting(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: greeting\n");
        return 1;
    }
    return start_chassis_action(CHASSIS_ACTION_GREETING, "greeting");
}

static int cmd_turn_left(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: turn_left\n");
        return 1;
    }
    return start_chassis_action(CHASSIS_ACTION_TURN_LEFT, "turn_left");
}

static int cmd_turn_right(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: turn_right\n");
        return 1;
    }
    return start_chassis_action(CHASSIS_ACTION_TURN_RIGHT, "turn_right");
}

static int cmd_reverse_park(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: reverse_park\n");
        return 1;
    }
    return start_chassis_action(CHASSIS_ACTION_REVERSE_PARK, "reverse_park");
}

static int cmd_spin_clockwise(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: spin_clockwise\n");
        return 1;
    }
    return start_chassis_action(CHASSIS_ACTION_SPIN_CLOCKWISE, "spin_clockwise");
}

static int cmd_spin_counterclockwise(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: spin_counterclockwise\n");
        return 1;
    }
    return start_chassis_action(CHASSIS_ACTION_SPIN_COUNTERCLOCKWISE,
                                "spin_counterclockwise");
}

static int cmd_emotion_happy(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: emotion_happy\n");
        return 1;
    }
    return start_chassis_action(CHASSIS_ACTION_EMOTION_HAPPY, "emotion_happy");
}

static int cmd_emotion_scared(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: emotion_scared\n");
        return 1;
    }
    return start_chassis_action(CHASSIS_ACTION_EMOTION_SCARED, "emotion_scared");
}

static int cmd_emotion_angry(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: emotion_angry\n");
        return 1;
    }
    return start_chassis_action(CHASSIS_ACTION_EMOTION_ANGRY, "emotion_angry");
}

static int cmd_dance_take_me_hand(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: dance_take_me_hand\n");
        return 1;
    }

    esp_err_t chassis_err = chassis_action_cancel();
    esp_err_t arm_err = arm_action_cancel();
    esp_err_t dance_err = dance_cancel();
    if (chassis_err != ESP_OK || arm_err != ESP_OK || dance_err != ESP_OK) {
        printf("Could not prepare dance: chassis=%s arm=%s dance=%s\n",
               esp_err_to_name(chassis_err), esp_err_to_name(arm_err),
               esp_err_to_name(dance_err));
        return 1;
    }

    esp_err_t err = dance_start_take_me_hand();
    if (err != ESP_OK) {
        printf("Could not start Take Me Hand dance: %s\n", esp_err_to_name(err));
        return 1;
    }
    printf("Take Me Hand dance started: 128 BPM, 32 bars, about 60.7 s\n");
    return 0;
}

static int cmd_dance_believer(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: dance_believer\n");
        return 1;
    }

    esp_err_t chassis_err = chassis_action_cancel();
    esp_err_t arm_err = arm_action_cancel();
    esp_err_t dance_err = dance_cancel();
    if (chassis_err != ESP_OK || arm_err != ESP_OK || dance_err != ESP_OK) {
        printf("Could not prepare dance: chassis=%s arm=%s dance=%s\n",
               esp_err_to_name(chassis_err), esp_err_to_name(arm_err),
               esp_err_to_name(dance_err));
        return 1;
    }

    esp_err_t err = dance_start_believer();
    if (err != ESP_OK) {
        printf("Could not start Believer dance: %s\n", esp_err_to_name(err));
        return 1;
    }
    printf("Believer dance started: 129.2 BPM, about 31.4 s plus IMU heading-return finale\n");
    return 0;
}

static int cmd_dance_new_believer(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: dance_new_believer\n");
        return 1;
    }

    esp_err_t chassis_err = chassis_action_cancel();
    esp_err_t arm_err = arm_action_cancel();
    esp_err_t dance_err = dance_cancel();
    if (chassis_err != ESP_OK || arm_err != ESP_OK || dance_err != ESP_OK) {
        printf("Could not prepare dance: chassis=%s arm=%s dance=%s\n",
               esp_err_to_name(chassis_err), esp_err_to_name(arm_err),
               esp_err_to_name(dance_err));
        return 1;
    }

    esp_err_t err = dance_start_new_believer();
    if (err != ESP_OK) {
        printf("Could not start New Believer dance: %s\n", esp_err_to_name(err));
        return 1;
    }
    printf("New Believer dance started: 128.7 BPM, 19 bars, about 35.9 s\n");
    return 0;
}

static int cmd_encoder(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: enc\n");
        return 1;
    }
    encoder_print_status();
    return 0;
}

static int cmd_encreset(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: encreset\n");
        return 1;
    }
    encoder_reset_all();
    printf("Encoder counts cleared\n");
    return 0;
}

static int cmd_cps(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: cps\n");
        return 1;
    }

    int left_start;
    int right_start;
    ESP_ERROR_CHECK(encoder_get_count(MOTOR_LEFT_INDEX + 1, &left_start));
    ESP_ERROR_CHECK(encoder_get_count(MOTOR_RIGHT_INDEX + 1, &right_start));
    int64_t start_us = esp_timer_get_time();

    vTaskDelay(pdMS_TO_TICKS(1000));

    int left_end;
    int right_end;
    ESP_ERROR_CHECK(encoder_get_count(MOTOR_LEFT_INDEX + 1, &left_end));
    ESP_ERROR_CHECK(encoder_get_count(MOTOR_RIGHT_INDEX + 1, &right_end));
    int64_t elapsed_us = esp_timer_get_time() - start_us;
    int left_delta = left_end - left_start;
    int right_delta = right_end - right_start;
    int left_cps = (int)((int64_t)left_delta * 1000000 / elapsed_us);
    int right_cps = (int)((int64_t)right_delta * 1000000 / elapsed_us);

    printf("1 s encoder sample (%lld ms): left=%d count, %d count/s; "
           "right=%d count, %d count/s\n",
           (long long)(elapsed_us / 1000), left_delta, left_cps,
           right_delta, right_cps);
    return 0;
}

static int cmd_cpslog(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: cpslog <start|stop|list|summary|dump ID|delete ID|clear|status>\n");
        return 1;
    }

    esp_err_t err;
    if (strcmp(argv[1], "start") == 0) {
        if (argc != 2) {
            goto usage;
        }
        err = cps_log_start();
        if (err == ESP_OK) {
            printf("CPS logging started: every %u ms (%u Hz), up to %u samples / 30 s\n",
                   CPS_LOG_SAMPLE_PERIOD_MS, 1000 / CPS_LOG_SAMPLE_PERIOD_MS,
                   CPS_LOG_MAX_SAMPLES);
        }
    } else if (strcmp(argv[1], "stop") == 0) {
        if (argc != 2) {
            goto usage;
        }
        size_t sample_count = 0;
        unsigned int session_id = 0;
        err = cps_log_stop_and_save(&sample_count, &session_id);
        if (err == ESP_OK) {
            printf("CPS log saved as ID %03u: %u samples\n", session_id,
                   (unsigned)sample_count);
        }
    } else if (strcmp(argv[1], "list") == 0) {
        if (argc != 2) {
            goto usage;
        }
        err = cps_log_list();
    } else if (strcmp(argv[1], "summary") == 0) {
        if (argc != 2) {
            goto usage;
        }
        err = cps_log_summary();
    } else if (strcmp(argv[1], "dump") == 0 || strcmp(argv[1], "delete") == 0) {
        int session_id;
        if (argc != 3 || !parse_int_range(argv[2], 0, CPS_LOG_MAX_SESSION_ID, &session_id)) {
            goto usage;
        }
        err = strcmp(argv[1], "dump") == 0 ? cps_log_dump((unsigned)session_id)
                                             : cps_log_delete((unsigned)session_id);
        if (err == ESP_OK && strcmp(argv[1], "delete") == 0) {
            printf("CPS log ID %03d deleted\n", session_id);
        }
    } else if (strcmp(argv[1], "clear") == 0) {
        if (argc != 2) {
            goto usage;
        }
        err = cps_log_clear();
        if (err == ESP_OK) {
            printf("All CPS logs cleared\n");
        }
    } else if (strcmp(argv[1], "status") == 0) {
        if (argc != 2) {
            goto usage;
        }
        cps_log_print_status();
        return 0;
    } else {
        goto usage;
    }

    if (err != ESP_OK) {
        printf("CPS log command failed: %s\n", esp_err_to_name(err));
        return 1;
    }
    return 0;

usage:
    printf("Usage: cpslog <start|stop|list|summary|dump ID|delete ID|clear|status>\n");
    return 1;
}

static int cmd_imulog(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: imulog <list|dump ID|delete ID|clear|status>\n");
        return 1;
    }

    esp_err_t err;
    if (strcmp(argv[1], "list") == 0) {
        if (argc != 2) {
            goto usage;
        }
        err = imu_spin_log_list();
    } else if (strcmp(argv[1], "dump") == 0 || strcmp(argv[1], "delete") == 0) {
        int session_id;
        if (argc != 3 || !parse_int_range(argv[2], 0, IMU_SPIN_LOG_MAX_SESSION_ID,
                                          &session_id)) {
            goto usage;
        }
        err = strcmp(argv[1], "dump") == 0 ? imu_spin_log_dump((unsigned)session_id)
                                             : imu_spin_log_delete((unsigned)session_id);
        if (err == ESP_OK && strcmp(argv[1], "delete") == 0) {
            printf("IMU log ID %03d deleted\n", session_id);
        }
    } else if (strcmp(argv[1], "clear") == 0) {
        if (argc != 2) {
            goto usage;
        }
        err = imu_spin_log_clear();
        if (err == ESP_OK) {
            printf("All IMU spin logs cleared\n");
        }
    } else if (strcmp(argv[1], "status") == 0) {
        if (argc != 2) {
            goto usage;
        }
        imu_spin_log_print_status();
        return 0;
    } else {
        goto usage;
    }

    if (err != ESP_OK) {
        printf("IMU log command failed: %s\n", esp_err_to_name(err));
        return 1;
    }
    return 0;

usage:
    printf("Usage: imulog <list|dump ID|delete ID|clear|status>\n");
    return 1;
}

static int cmd_encdbg(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: encdbg\n");
        return 1;
    }
    encoder_debug_levels();
    return 0;
}

static int cmd_battery(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: battery\n");
        return 1;
    }
    battery_print_status();
    return 0;
}

static bool parse_u8(const char *text, uint8_t *value)
{
    char *end = NULL;
    errno = 0;
    long parsed = strtol(text, &end, 0);
    if (errno == ERANGE || end == text || *end != '\0' || parsed < 0 || parsed > UINT8_MAX) {
        return false;
    }
    *value = (uint8_t)parsed;
    return true;
}

static void print_imu_sample(const imu_lsm6ds3tr_sample_t *sample, unsigned int index)
{
    int32_t temperature_abs = sample->temperature_mC < 0 ? -sample->temperature_mC
                                                          : sample->temperature_mC;
    int temperature_whole = (int)(temperature_abs / 1000);
    int temperature_fraction = (int)(temperature_abs % 1000);
    printf("IMU sample %u: temp=%s%d.%03d C, gyro_raw=[%d, %d, %d], "
           "accel_raw=[%d, %d, %d]\n",
           index, sample->temperature_mC < 0 ? "-" : "", temperature_whole,
           temperature_fraction, sample->gyro_x, sample->gyro_y, sample->gyro_z,
           sample->accel_x, sample->accel_y, sample->accel_z);
}

static int cmd_imu(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "status") == 0) {
        imu_print_status();
        imu_heater_print_status();
        return imu_is_ready() ? 0 : 1;
    }

    if (argc == 2 && strcmp(argv[1], "reinit") == 0) {
        esp_err_t err = imu_reinit();
        if (err != ESP_OK) {
            printf("IMU reinitialization failed: %s\n", esp_err_to_name(err));
            return 1;
        }
        esp_err_t heater_err = imu_heater_init();
        if (heater_err != ESP_OK) {
            printf("IMU heater start failed: %s\n", esp_err_to_name(heater_err));
            return 1;
        }
        imu_print_status();
        imu_heater_print_status();
        return 0;
    }

    if (argc == 2 && strcmp(argv[1], "calibrate") == 0) {
        printf("Keep the robot still for about one second...\n");
        esp_err_t err = chassis_action_calibrate_imu();
        if (err != ESP_OK) {
            printf("IMU spin calibration failed: %s\n", esp_err_to_name(err));
            return 1;
        }
        printf("IMU spin calibration saved; future spin commands start immediately\n");
        return 0;
    }

    if (argc == 3 && strcmp(argv[1], "reg") == 0) {
        uint8_t reg;
        if (!parse_u8(argv[2], &reg)) {
            printf("Usage: imu reg <0x00..0xFF>\n");
            return 1;
        }
        uint8_t value;
        esp_err_t err = imu_read_register(reg, &value);
        if (err != ESP_OK) {
            printf("IMU register read failed: %s\n", esp_err_to_name(err));
            return 1;
        }
        printf("IMU reg 0x%02X = 0x%02X\n", reg, value);
        return 0;
    }

    unsigned int sample_count = 1;
    if (argc == 3 && strcmp(argv[1], "sample") == 0) {
        int parsed_count;
        if (!parse_int_range(argv[2], 1, 100, &parsed_count)) {
            printf("Usage: imu sample [count 1..100]\n");
            return 1;
        }
        sample_count = (unsigned int)parsed_count;
    } else if (argc != 1 && !(argc == 2 && strcmp(argv[1], "sample") == 0)) {
        printf("Usage: imu <status|sample [count 1..100]|reg <0x00..0xFF>|reinit|calibrate>\n");
        return 1;
    }

    for (unsigned int i = 0; i < sample_count; ++i) {
        imu_lsm6ds3tr_sample_t sample;
        esp_err_t err = imu_read_sample(&sample);
        if (err != ESP_OK) {
            printf("IMU sample failed: %s\n", esp_err_to_name(err));
            return 1;
        }
        print_imu_sample(&sample, i + 1);
        if (i + 1 < sample_count) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    return 0;
}

static int cmd_servo(int argc, char **argv)
{
    int servo_id;
    int angle;
    if (argc != 3 ||
        !parse_int_range(argv[1], 1, SERVO_COUNT, &servo_id) ||
        !parse_int_range(argv[2], SERVO_ANGLE_MIN, SERVO_ANGLE_MAX, &angle)) {
        printf("Usage: servo <1=right/J1|2=left/J5> <angle %d..%d>\n", SERVO_ANGLE_MIN,
               SERVO_ANGLE_MAX);
        return 1;
    }

    esp_err_t dance_err = dance_cancel();
    if (dance_err != ESP_OK) {
        printf("Could not cancel dance: %s\n", esp_err_to_name(dance_err));
        return 1;
    }
    esp_err_t arm_err = arm_action_cancel();
    if (arm_err != ESP_OK) {
        printf("Could not cancel arm action: %s\n", esp_err_to_name(arm_err));
        return 1;
    }
    ESP_ERROR_CHECK(servo_set_angle(servo_id, angle));
    printf("Servo %d set to %d degrees\n", servo_id, angle);
    return 0;
}

static int cmd_servooff(int argc, char **argv)
{
    int servo_id;
    if (argc != 2 || !parse_int_range(argv[1], 1, SERVO_COUNT, &servo_id)) {
        printf("Usage: servooff <1=right/J1|2=left/J5>\n");
        return 1;
    }

    esp_err_t dance_err = dance_cancel();
    if (dance_err != ESP_OK) {
        printf("Could not cancel dance: %s\n", esp_err_to_name(dance_err));
        return 1;
    }
    esp_err_t arm_err = arm_action_cancel();
    if (arm_err != ESP_OK) {
        printf("Could not cancel arm action: %s\n", esp_err_to_name(arm_err));
        return 1;
    }
    ESP_ERROR_CHECK(servo_off(servo_id));
    printf("Servo %d PWM disabled\n", servo_id);
    return 0;
}

static bool parse_tunable_chassis_action(const char *name, chassis_action_id_t *action)
{
    if (strcmp(name, "move_forward") == 0) {
        *action = CHASSIS_ACTION_MOVE_FORWARD;
    } else if (strcmp(name, "move_backward") == 0) {
        *action = CHASSIS_ACTION_MOVE_BACKWARD;
    } else if (strcmp(name, "greeting") == 0) {
        *action = CHASSIS_ACTION_GREETING;
    } else if (strcmp(name, "turn_left") == 0) {
        *action = CHASSIS_ACTION_TURN_LEFT;
    } else if (strcmp(name, "turn_right") == 0) {
        *action = CHASSIS_ACTION_TURN_RIGHT;
    } else if (strcmp(name, "reverse_park") == 0) {
        *action = CHASSIS_ACTION_REVERSE_PARK;
    } else if (strcmp(name, "spin_clockwise") == 0) {
        *action = CHASSIS_ACTION_SPIN_CLOCKWISE;
    } else if (strcmp(name, "spin_counterclockwise") == 0) {
        *action = CHASSIS_ACTION_SPIN_COUNTERCLOCKWISE;
    } else if (strcmp(name, "emotion_happy") == 0) {
        *action = CHASSIS_ACTION_EMOTION_HAPPY;
    } else if (strcmp(name, "emotion_scared") == 0) {
        *action = CHASSIS_ACTION_EMOTION_SCARED;
    } else {
        return false;
    }
    return true;
}

static bool parse_tunable_arm_pose(const char *name, arm_tune_pose_t *pose)
{
    if (strcmp(name, "home") == 0) {
        *pose = ARM_TUNE_POSE_HOME;
    } else if (strcmp(name, "open") == 0) {
        *pose = ARM_TUNE_POSE_OPEN;
    } else if (strcmp(name, "open_swing") == 0) {
        *pose = ARM_TUNE_POSE_OPEN_SWING;
    } else if (strcmp(name, "raised") == 0) {
        *pose = ARM_TUNE_POSE_RAISED;
    } else if (strcmp(name, "handshake_ready") == 0) {
        *pose = ARM_TUNE_POSE_HANDSHAKE_READY;
    } else if (strcmp(name, "handshake_high") == 0) {
        *pose = ARM_TUNE_POSE_HANDSHAKE_HIGH;
    } else if (strcmp(name, "handshake_low") == 0) {
        *pose = ARM_TUNE_POSE_HANDSHAKE_LOW;
    } else if (strcmp(name, "wave_ready") == 0) {
        *pose = ARM_TUNE_POSE_WAVE_READY;
    } else if (strcmp(name, "wave_high") == 0) {
        *pose = ARM_TUNE_POSE_WAVE_HIGH;
    } else if (strcmp(name, "wave_low") == 0) {
        *pose = ARM_TUNE_POSE_WAVE_LOW;
    } else if (strcmp(name, "happy_a") == 0) {
        *pose = ARM_TUNE_POSE_HAPPY_A;
    } else if (strcmp(name, "happy_b") == 0) {
        *pose = ARM_TUNE_POSE_HAPPY_B;
    } else if (strcmp(name, "park_a") == 0) {
        *pose = ARM_TUNE_POSE_REVERSE_PARK_A;
    } else if (strcmp(name, "park_b") == 0) {
        *pose = ARM_TUNE_POSE_REVERSE_PARK_B;
    } else {
        return false;
    }
    return true;
}

static bool tuning_targets_are_idle(void)
{
    return !dance_is_active() && !chassis_action_is_active() && !arm_action_is_active();
}

static int cmd_tune(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "show") == 0) {
        printf("Tunable chassis actions: move_forward move_backward greeting turn_left turn_right\n");
        printf("                         reverse_park spin_clockwise spin_counterclockwise\n");
        printf("                         emotion_happy emotion_scared\n");
        chassis_action_tune_print();
        arm_action_tune_print();
        return 0;
    }

    if (!tuning_targets_are_idle()) {
        printf("Tuning is only allowed while dance, chassis, and arm actions are idle; use stop first\n");
        return 1;
    }

    if (argc == 2 && strcmp(argv[1], "reset") == 0) {
        esp_err_t chassis_err = chassis_action_tune_reset();
        esp_err_t arm_err = arm_action_tune_reset();
        if (chassis_err != ESP_OK || arm_err != ESP_OK) {
            printf("Could not reset tuning: chassis=%s arm=%s\n", esp_err_to_name(chassis_err),
                   esp_err_to_name(arm_err));
            return 1;
        }
        printf("Runtime tuning reset to firmware defaults; saved reverse_park speed cleared\n");
        return 0;
    }

    if (argc == 3 && strcmp(argv[1], "save") == 0) {
        if (strcmp(argv[2], "reverse_park") != 0) {
            printf("Usage: tune save reverse_park\n");
            return 1;
        }
        esp_err_t err = chassis_action_tune_save_reverse_park();
        if (err != ESP_OK) {
            printf("Could not save reverse_park PID targets: %s\n", esp_err_to_name(err));
            return 1;
        }
        printf("Saved reverse_park PID targets; they will be loaded after reboot\n");
        return 0;
    }

    if (argc == 5 && strcmp(argv[1], "speed") == 0) {
        chassis_action_id_t action;
        int left_speed;
        int right_speed;
        if (!parse_tunable_chassis_action(argv[2], &action) ||
            !parse_int_range(argv[3], -100, 100, &left_speed) ||
            !parse_int_range(argv[4], -100, 100, &right_speed)) {
            printf("Usage: tune speed <action> <left -100..100> <right -100..100>\n");
            return 1;
        }
        esp_err_t err = chassis_action_tune_set_speed(action, left_speed, right_speed);
        if (err != ESP_OK) {
            printf("Could not tune chassis PID targets: %s\n", esp_err_to_name(err));
            return 1;
        }
        printf("Tuned %s PID targets: left=%d right=%d (RAM only%s)\n", argv[2], left_speed,
               right_speed, action == CHASSIS_ACTION_REVERSE_PARK
                                ? "; run 'tune save reverse_park' to persist"
                                : "");
        return 0;
    }

    if (argc == 5 && strcmp(argv[1], "arm") == 0) {
        arm_tune_pose_t pose;
        int right_angle;
        int left_angle;
        if (!parse_tunable_arm_pose(argv[2], &pose) ||
            !parse_int_range(argv[3], SERVO_ANGLE_MIN, SERVO_ANGLE_MAX, &right_angle) ||
            !parse_int_range(argv[4], SERVO_ANGLE_MIN, SERVO_ANGLE_MAX, &left_angle)) {
            printf("Usage: tune arm <pose> <servo1/right 0..180> <servo2/left 0..180>\n");
            return 1;
        }
        esp_err_t err = arm_action_tune_set_pose(pose, right_angle, left_angle);
        if (err != ESP_OK) {
            printf("Could not tune arm pose: %s\n", esp_err_to_name(err));
            return 1;
        }
        printf("Tuned %s: Servo 1/right=%d Servo 2/left=%d (RAM only)\n", argv[2],
               right_angle, left_angle);
        return 0;
    }

    printf("Usage:\n");
    printf("  tune speed <action> <left -100..100> <right -100..100>\n");
    printf("  tune save reverse_park\n");
    printf("  tune arm <pose> <servo1/right 0..180> <servo2/left 0..180>\n");
    printf("  tune <show|reset>\n");
    printf("Use 'tune show' for action and pose names. Only saved reverse_park speed persists.\n");
    return 1;
}

static int start_arm_action(arm_action_id_t action, const char *name)
{
    esp_err_t dance_err = dance_cancel();
    if (dance_err != ESP_OK) {
        printf("Could not cancel dance: %s\n", esp_err_to_name(dance_err));
        return 1;
    }
    esp_err_t err = arm_action_start(action);
    if (err == ESP_ERR_INVALID_STATE) {
        printf("Another arm action is running; use armstop or stop to cancel it\n");
        return 1;
    }
    if (err != ESP_OK) {
        printf("Could not start %s: %s\n", name, esp_err_to_name(err));
        return 1;
    }
    printf("%s started; use armstop or stop to cancel\n", name);
    return 0;
}

static int cmd_handshake(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: handshake\n");
        return 1;
    }
    return start_arm_action(ARM_ACTION_HANDSHAKE, "Handshake");
}

static int cmd_wave_hand(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: wave_hand\n");
        return 1;
    }
    return start_arm_action(ARM_ACTION_WAVE_HAND, "Wave hand");
}

static int cmd_open_arms(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: open_arms\n");
        return 1;
    }
    return start_arm_action(ARM_ACTION_OPEN_ARMS, "Open arms");
}

static int cmd_raise_both_arms(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: raise_both_arms\n");
        return 1;
    }
    return start_arm_action(ARM_ACTION_RAISE_BOTH_ARMS, "Raise both arms");
}

static int cmd_close_arms(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: close_arms\n");
        return 1;
    }
    return start_arm_action(ARM_ACTION_CLOSE_ARMS, "Close arms");
}

static int cmd_armstop(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: armstop\n");
        return 1;
    }

    esp_err_t err = arm_action_cancel();
    if (err != ESP_OK) {
        printf("Arm action cancellation failed: %s\n", esp_err_to_name(err));
        return 1;
    }
    printf("Arm action cancelled; current arm positions kept\n");
    return 0;
}

static int cmd_rgb(int argc, char **argv)
{
    int red;
    int green;
    int blue;
    if (argc != 4 || !parse_int_range(argv[1], 0, 255, &red) ||
        !parse_int_range(argv[2], 0, 255, &green) ||
        !parse_int_range(argv[3], 0, 255, &blue)) {
        printf("Usage: rgb <red 0..255> <green 0..255> <blue 0..255>\n");
        return 1;
    }

    ESP_ERROR_CHECK(rgb_led_fill((uint8_t)red, (uint8_t)green, (uint8_t)blue));
    return 0;
}

static int cmd_rainbow(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: rainbow\n");
        return 1;
    }
    ESP_ERROR_CHECK(rgb_led_rainbow());
    return 0;
}

static int cmd_rgboff(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: rgboff\n");
        return 1;
    }
    ESP_ERROR_CHECK(rgb_led_off());
    return 0;
}

/*
 * One firmware image is flashed to every board; this command gives each unit its
 * own BLE identity. The name is stored in NVS, so it survives power cycles and
 * later app reflashes.
 */
static int cmd_btname(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "show") == 0) {
        printf("BLE name: %s (%s)\n", ble_motor_device_name(),
               ble_motor_device_name_is_saved() ? "saved in NVS" : "default from MAC");
        printf("MAC-derived default: %s\n", ble_motor_device_name_default());
        return 0;
    }

    if (argc == 3 && strcmp(argv[1], "set") == 0) {
        esp_err_t err = ble_motor_device_name_set(argv[2]);
        if (err == ESP_ERR_INVALID_ARG) {
            printf("Invalid name; use 1-%d characters from A-Z a-z 0-9 . _ -\n",
                   BLE_MOTOR_DEVICE_NAME_MAX_LEN);
            return 1;
        }
        if (err != ESP_OK) {
            printf("Could not save the BLE name: %s\n", esp_err_to_name(err));
            return 1;
        }
        printf("BLE name saved: %s (kept across power cycles)\n", ble_motor_device_name());
        return 0;
    }

    if (argc == 2 && strcmp(argv[1], "reset") == 0) {
        esp_err_t err = ble_motor_device_name_reset();
        if (err != ESP_OK) {
            printf("Could not reset the BLE name: %s\n", esp_err_to_name(err));
            return 1;
        }
        printf("BLE name reset to the default: %s\n", ble_motor_device_name());
        return 0;
    }

    printf("Usage:\n");
    printf("  btname set <name>   Save a new BLE name (1-%d chars: A-Z a-z 0-9 . _ -)\n",
           BLE_MOTOR_DEVICE_NAME_MAX_LEN);
    printf("  btname show         Show the current BLE name\n");
    printf("  btname reset        Restore the MAC-derived default %s\n",
           ble_motor_device_name_default());
    printf("A saved name is reused after every reboot; reconnect the central to see it.\n");
    return 1;
}

static int cmd_status(int argc, char **argv)
{
    if (argc != 1) {
        printf("Usage: status\n");
        return 1;
    }

    printf("Firmware v%s\n", esp_app_get_description()->version);
    motor_control_print_status();
    speed_control_print_status();
    chassis_action_print_status();
    printf("Encoders:\n");
    encoder_print_status();
    battery_print_status();
    printf("Servos:\n");
    servo_print_status();
    arm_action_print_status();
    imu_print_status();
    imu_heater_print_status();
    imu_spin_log_print_status();
    printf("BLE device: %s (%s)\n", ble_motor_device_name(),
           ble_motor_device_name_is_saved() ? "saved in NVS" : "default from MAC");
    return 0;
}

static void register_motor_commands(void)
{
    ESP_ERROR_CHECK(esp_console_register_help_command());

    const esp_console_cmd_t commands[] = {
        {
            .command = "motor",
            .help = "Set one motor: motor <1-2> <-100..100>",
            .func = cmd_motor,
        },
        {
            .command = "motorinv",
            .help = "Calibrate polarity: motorinv <1-2> <0|1>",
            .func = cmd_motorinv,
        },
        {
            .command = "drive",
            .help = "Set chassis sides: drive <left -100..100> <right -100..100>",
            .func = cmd_drive,
        },
        {
            .command = "speed",
            .help = "PID velocity: speed <left -100..100> <right -100..100>",
            .func = cmd_speed,
        },
        {
            .command = "tune",
            .help = "Action tuning: tune <speed|save|arm|show|reset>; reverse_park save persists",
            .func = cmd_tune,
        },
        {
            .command = "move_forward",
            .help = "IMU heading-hold PID drive +55/+55 for one second",
            .func = cmd_move_forward,
        },
        {
            .command = "move_backward",
            .help = "IMU heading-hold PID drive -55/-55 for one second",
            .func = cmd_move_backward,
        },
        {
            .command = "greeting",
            .help = "Wave right arm and PID drive +55/+55 for three seconds",
            .func = cmd_greeting,
        },
        {
            .command = "turn_left",
            .help = "PID left=0 right=+70 for 1.5 s, then stop",
            .func = cmd_turn_left,
        },
        {
            .command = "turn_right",
            .help = "PID left=+70 right=0 for 1.5 s, then stop",
            .func = cmd_turn_right,
        },
        {
            .command = "reverse_park",
            .help = "Fan-shaped reverse: left=-50 right=-100 until IMU reaches 90 degrees",
            .func = cmd_reverse_park,
        },
        {
            .command = "spin_clockwise",
            .help = "PID left=+100 right=-100 for one IMU-measured turn",
            .func = cmd_spin_clockwise,
        },
        {
            .command = "spin_counterclockwise",
            .help = "PID left=-100 right=+100 for one IMU-measured turn",
            .func = cmd_spin_counterclockwise,
        },
        {
            .command = "emotion_happy",
            .help = "Rapid arm raise/two swings plus one IMU-measured spin",
            .func = cmd_emotion_happy,
        },
        {
            .command = "emotion_scared",
            .help = "Rapid arm close plus PID -80/-80 for 500 ms",
            .func = cmd_emotion_scared,
        },
        {
            .command = "emotion_angry",
            .help = "Arm open/close twice plus four rapid PID wobble segments",
            .func = cmd_emotion_angry,
        },
        {
            .command = "dance_take_me_hand",
            .help = "Run the 128 BPM Take Me Hand choreography for about 60.7 seconds",
            .func = cmd_dance_take_me_hand,
        },
        {
            .command = "dance_believer",
            .help = "Run the 129.2 BPM Believer choreography for about 31.4 s plus IMU finale",
            .func = cmd_dance_believer,
        },
        {
            .command = "dance_new_believer",
            .help = "Run the 128.7 BPM New Believer choreography for about 35.9 seconds",
            .func = cmd_dance_new_believer,
        },
        {
            .command = "forward",
            .help = "Drive forward: forward [speed 1..100]",
            .func = cmd_forward,
        },
        {
            .command = "backward",
            .help = "Drive backward: backward [speed 1..100]",
            .func = cmd_backward,
        },
        {
            .command = "left",
            .help = "Turn left in place: left [speed 1..100]",
            .func = cmd_left,
        },
        {
            .command = "right",
            .help = "Turn right in place: right [speed 1..100]",
            .func = cmd_right,
        },
        {
            .command = "stop",
            .help = "Immediately stop all motors",
            .func = cmd_stop,
        },
        {
            .command = "enc",
            .help = "Show M1/M2 encoder counts",
            .func = cmd_encoder,
        },
        {
            .command = "encreset",
            .help = "Clear both encoder counts",
            .func = cmd_encreset,
        },
        {
            .command = "cps",
            .help = "Measure left/right encoder count over one second",
            .func = cmd_cps,
        },
        {
            .command = "cpslog",
            .help = "Save/summarize/dump 30 s CPS logs: cpslog <start|stop|list|summary|dump ID|delete ID|clear|status>",
            .func = cmd_cpslog,
        },
        {
            .command = "imulog",
            .help = "Spin IMU logs: imulog <list|dump ID|delete ID|clear|status>",
            .func = cmd_imulog,
        },
        {
            .command = "encdbg",
            .help = "Show live encoder pin levels (A/B) for diagnosis",
            .func = cmd_encdbg,
        },
        {
            .command = "battery",
            .help = "Read the J2 battery voltage",
            .func = cmd_battery,
        },
        {
            .command = "imu",
            .help = "LSM6DS3TR-C: imu <status|sample [count]|reg <addr>|reinit|calibrate>",
            .func = cmd_imu,
        },
        {
            .command = "servo",
            .help = "Set arm: servo <1=right/J1|2=left/J5> <0..180>",
            .func = cmd_servo,
        },
        {
            .command = "servooff",
            .help = "Disable arm PWM: servooff <1=right/J1|2=left/J5>",
            .func = cmd_servooff,
        },
        {
            .command = "handshake",
            .help = "Right arm handshake: 70 deg, 3 swings, then return to 0 deg",
            .func = cmd_handshake,
        },
        {
            .command = "wave_hand",
            .help = "Right arm wave: 135 deg, 4 swings between 145/125 deg, then return to 0 deg",
            .func = cmd_wave_hand,
        },
        {
            .command = "open_arms",
            .help = "Open both arms, then make two small welcome swings",
            .func = cmd_open_arms,
        },
        {
            .command = "raise_both_arms",
            .help = "Raise both arms: right=130 deg, left=50 deg",
            .func = cmd_raise_both_arms,
        },
        {
            .command = "close_arms",
            .help = "Close both arms toward phone: right=0 deg, left=180 deg",
            .func = cmd_close_arms,
        },
        {
            .command = "armstop",
            .help = "Cancel active arm preset and keep its current pose",
            .func = cmd_armstop,
        },
        {
            .command = "rgb",
            .help = "Set all RGB pixels: rgb <r> <g> <b>",
            .func = cmd_rgb,
        },
        {
            .command = "rainbow",
            .help = "Show an RGB test pattern",
            .func = cmd_rainbow,
        },
        {
            .command = "rgboff",
            .help = "Turn all RGB pixels off",
            .func = cmd_rgboff,
        },
        {
            .command = "btname",
            .help = "Persistent BLE name: btname <set <name>|show|reset>",
            .func = cmd_btname,
        },
        {
            .command = "status",
            .help = "Show firmware version and all peripheral state",
            .func = cmd_status,
        },
    };

    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&commands[i]));
    }
}

static void start_usb_console(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "robot> ";
    repl_config.max_cmdline_length = 128;

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t usb_config =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usb_config,
                                                          &repl_config, &repl));
#else
#error "robot_chassis requires USB Serial/JTAG as the primary console"
#endif

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting RobotPhone chassis controller, firmware v%s",
             esp_app_get_description()->version);
    initialize_nvs();
    ESP_ERROR_CHECK(motor_control_init());
    ESP_ERROR_CHECK(encoder_init());
    ESP_ERROR_CHECK(speed_control_init());
    ESP_ERROR_CHECK(imu_spin_log_init());
    ESP_ERROR_CHECK(chassis_action_init());

    /* IMU absence must not prevent safe motor/servo operation during board bring-up. */
    esp_err_t imu_err = imu_init();
    if (imu_err != ESP_OK) {
        ESP_LOGW(TAG, "IMU unavailable (%s); use 'imu reinit' after checking the I2C bus",
                 esp_err_to_name(imu_err));
    } else {
        ESP_LOGI(TAG, "Keep robot still: refreshing IMU gyro zero bias for spin actions");
        esp_err_t calibration_err = chassis_action_calibrate_imu();
        if (calibration_err != ESP_OK) {
            ESP_LOGW(TAG, "IMU spin calibration failed (%s); run 'imu calibrate' while still",
                     esp_err_to_name(calibration_err));
        }
        esp_err_t heater_err = imu_heater_init();
        if (heater_err != ESP_OK) {
            ESP_LOGW(TAG, "IMU heater unavailable (%s); IMU remains usable", esp_err_to_name(heater_err));
        }
    }

    ESP_ERROR_CHECK(servo_init());
    ESP_ERROR_CHECK(arm_action_init());
    ESP_ERROR_CHECK(dance_init());
    ESP_ERROR_CHECK(rgb_led_init());

    /* Battery sense is non-critical: keep the rest of the chassis up if the ADC
     * cannot be configured. */
    esp_err_t battery_err = battery_init();
    if (battery_err != ESP_OK) {
        ESP_LOGW(TAG, "Battery ADC unavailable (%s)", esp_err_to_name(battery_err));
    }

    register_motor_commands();
    start_usb_console();

    /* Start BLE last so the console command engine is fully initialized before
     * any BLE write can be dispatched to it (BLE shares the shell command set). */
    ESP_ERROR_CHECK(ble_motor_start());

    ESP_LOGI(TAG, "USB shell: idf.py -p /dev/ttyACM0 monitor");
    ESP_LOGI(TAG, "BLE device name: %s (%s)", ble_motor_device_name(),
             ble_motor_device_name_is_saved() ? "saved in NVS" : "default from MAC");
    ESP_LOGI(TAG, "BLE shares the shell command set plus M|left|right|$; "
                  "rename with 'btname set <name>'");
    ESP_LOGI(TAG, "Type 'help' for commands; use stop to halt motion");
}
