#include "imu_spin_log.h"

#include <dirent.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* SPIFFS is mounted by cps_log_init() before imu_spin_log_init() is called. */
#define IMU_SPIN_LOG_MAGIC 0x494D554CU /* "IMUL" */
#define IMU_SPIN_LOG_VERSION_LEGACY 1U
#define IMU_SPIN_LOG_VERSION 2U
#define IMU_SPIN_LOG_BASE_PATH "/spiffs"
#define IMU_SPIN_LOG_PATH_SIZE 40

typedef struct __attribute__((packed)) {
    uint32_t elapsed_ms;
    float turn_rate_dps;
    float turned_degrees;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int32_t temperature_mC;
} imu_spin_log_legacy_sample_t;

typedef struct __attribute__((packed)) {
    uint32_t elapsed_ms;
    float turn_rate_dps;
    float turned_degrees;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int32_t temperature_mC;
    int16_t left_target_cps;
    int16_t right_target_cps;
    int16_t left_measured_cps;
    int16_t right_measured_cps;
    int8_t left_pwm_percent;
    int8_t right_pwm_percent;
} imu_spin_log_sample_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t sample_count;
    int8_t left_speed;
    int8_t right_speed;
    uint16_t reserved;
} imu_spin_log_header_t;

typedef struct {
    imu_spin_log_header_t header;
    imu_spin_log_sample_t samples[IMU_SPIN_LOG_MAX_SAMPLES];
} imu_spin_log_session_t;

_Static_assert(sizeof(imu_spin_log_legacy_sample_t) == 22,
               "unexpected legacy IMU spin log sample size");
_Static_assert(sizeof(imu_spin_log_sample_t) == 32, "unexpected IMU spin log sample size");
_Static_assert(sizeof(imu_spin_log_session_t) == 38412, "unexpected IMU spin log RAM footprint");

static imu_spin_log_session_t s_session;
static SemaphoreHandle_t s_mutex;
static int64_t s_start_time_us;
static bool s_recording;
static bool s_full;

static size_t sample_size(uint16_t version)
{
    if (version == IMU_SPIN_LOG_VERSION_LEGACY) {
        return sizeof(imu_spin_log_legacy_sample_t);
    }
    if (version == IMU_SPIN_LOG_VERSION) {
        return sizeof(imu_spin_log_sample_t);
    }
    return 0;
}

static size_t session_length(const imu_spin_log_header_t *header)
{
    size_t entry_size = sample_size(header->version);
    return entry_size == 0 ? 0 : sizeof(*header) + (size_t)header->sample_count * entry_size;
}

static bool header_is_valid(const imu_spin_log_header_t *header, size_t length)
{
    return header->magic == IMU_SPIN_LOG_MAGIC && sample_size(header->version) != 0 &&
           header->sample_count > 0 && header->sample_count <= IMU_SPIN_LOG_MAX_SAMPLES &&
           length == session_length(header);
}

static bool read_sample(FILE *file, const imu_spin_log_header_t *header,
                        imu_spin_log_sample_t *sample)
{
    size_t entry_size = sample_size(header->version);
    if (entry_size == 0) {
        return false;
    }
    memset(sample, 0, sizeof(*sample));
    return fread(sample, entry_size, 1, file) == 1;
}

static void session_path(unsigned int session_id, char *path, size_t path_size)
{
    snprintf(path, path_size, IMU_SPIN_LOG_BASE_PATH "/imu_%03u.bin", session_id);
}

static bool parse_session_id(const char *name, unsigned int *session_id)
{
    char extra;
    unsigned int parsed_id;
    if (sscanf(name, "imu_%u.bin%c", &parsed_id, &extra) != 1 ||
        parsed_id > IMU_SPIN_LOG_MAX_SESSION_ID) {
        return false;
    }
    *session_id = parsed_id;
    return true;
}

static esp_err_t find_free_session_id(unsigned int *session_id)
{
    char path[IMU_SPIN_LOG_PATH_SIZE];
    struct stat file_info;
    for (unsigned int id = 0; id <= IMU_SPIN_LOG_MAX_SESSION_ID; ++id) {
        session_path(id, path, sizeof(path));
        if (stat(path, &file_info) != 0) {
            if (errno == ENOENT) {
                *session_id = id;
                return ESP_OK;
            }
            return ESP_FAIL;
        }
    }
    return ESP_ERR_NO_MEM;
}

static esp_err_t read_header(FILE *file, imu_spin_log_header_t *header)
{
    if (fseek(file, 0, SEEK_END) != 0) {
        return ESP_FAIL;
    }
    long length = ftell(file);
    if (length < 0 || fseek(file, 0, SEEK_SET) != 0 ||
        fread(header, sizeof(*header), 1, file) != 1) {
        return ESP_FAIL;
    }
    return header_is_valid(header, (size_t)length) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t save_session_locked(unsigned int *session_id)
{
    unsigned int new_session_id;
    esp_err_t err = find_free_session_id(&new_session_id);
    if (err != ESP_OK) {
        return err;
    }

    char path[IMU_SPIN_LOG_PATH_SIZE];
    char temporary_path[IMU_SPIN_LOG_PATH_SIZE];
    session_path(new_session_id, path, sizeof(path));
    snprintf(temporary_path, sizeof(temporary_path), IMU_SPIN_LOG_BASE_PATH "/imu_%03u.tmp",
             new_session_id);
    FILE *file = fopen(temporary_path, "wb");
    if (file == NULL) {
        return ESP_FAIL;
    }

    size_t length = session_length(&s_session.header);
    size_t written = fwrite(&s_session, 1, length, file);
    int close_result = fclose(file);
    if (written != length || close_result != 0) {
        unlink(temporary_path);
        return ESP_FAIL;
    }
    if (rename(temporary_path, path) != 0) {
        unlink(temporary_path);
        return ESP_FAIL;
    }

    *session_id = new_session_id;
    return ESP_OK;
}

esp_err_t imu_spin_log_init(void)
{
    if (s_mutex != NULL) {
        return ESP_OK;
    }
    s_mutex = xSemaphoreCreateMutex();
    return s_mutex == NULL ? ESP_ERR_NO_MEM : ESP_OK;
}

esp_err_t imu_spin_log_start(int left_speed, int right_speed)
{
    if (s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_recording) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    memset(&s_session.header, 0, sizeof(s_session.header));
    s_session.header.magic = IMU_SPIN_LOG_MAGIC;
    s_session.header.version = IMU_SPIN_LOG_VERSION;
    s_session.header.left_speed = (int8_t)left_speed;
    s_session.header.right_speed = (int8_t)right_speed;
    s_start_time_us = esp_timer_get_time();
    s_recording = true;
    s_full = false;
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

void imu_spin_log_append(int64_t timestamp_us, const imu_lsm6ds3tr_sample_t *sample,
                         float turn_rate_dps, float turned_degrees,
                         const speed_control_telemetry_t *speed_telemetry)
{
    if (s_mutex == NULL || sample == NULL) {
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (!s_recording) {
        xSemaphoreGive(s_mutex);
        return;
    }
    if (s_session.header.sample_count >= IMU_SPIN_LOG_MAX_SAMPLES) {
        s_recording = false;
        s_full = true;
        xSemaphoreGive(s_mutex);
        return;
    }

    imu_spin_log_sample_t *entry = &s_session.samples[s_session.header.sample_count++];
    memset(entry, 0, sizeof(*entry));
    entry->elapsed_ms = (uint32_t)((timestamp_us - s_start_time_us) / 1000);
    entry->turn_rate_dps = turn_rate_dps;
    entry->turned_degrees = turned_degrees;
    entry->gyro_x = sample->gyro_x;
    entry->gyro_y = sample->gyro_y;
    entry->gyro_z = sample->gyro_z;
    entry->temperature_mC = sample->temperature_mC;
    if (speed_telemetry != NULL) {
        entry->left_target_cps = (int16_t)speed_telemetry->left_target_cps;
        entry->right_target_cps = (int16_t)speed_telemetry->right_target_cps;
        entry->left_measured_cps = (int16_t)speed_telemetry->left_measured_cps;
        entry->right_measured_cps = (int16_t)speed_telemetry->right_measured_cps;
        entry->left_pwm_percent = (int8_t)speed_telemetry->left_pwm_percent;
        entry->right_pwm_percent = (int8_t)speed_telemetry->right_pwm_percent;
    }
    if (s_session.header.sample_count == IMU_SPIN_LOG_MAX_SAMPLES) {
        s_recording = false;
        s_full = true;
    }
    xSemaphoreGive(s_mutex);
}

esp_err_t imu_spin_log_stop_and_save(size_t *sample_count, unsigned int *session_id)
{
    if (s_mutex == NULL || sample_count == NULL || session_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_session.header.sample_count == 0) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    s_recording = false;
    esp_err_t err = save_session_locked(session_id);
    if (err == ESP_OK) {
        *sample_count = s_session.header.sample_count;
        s_full = false;
    }
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t imu_spin_log_list(void)
{
    if (s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    DIR *directory = opendir(IMU_SPIN_LOG_BASE_PATH);
    if (directory == NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_FAIL;
    }

    printf("Saved IMU spin logs (id, samples, duration_ms, left, right, final_deg, peak_dps):\n");
    size_t listed = 0;
    struct dirent *entry;
    while ((entry = readdir(directory)) != NULL) {
        unsigned int session_id;
        if (!parse_session_id(entry->d_name, &session_id)) {
            continue;
        }

        char path[IMU_SPIN_LOG_PATH_SIZE];
        session_path(session_id, path, sizeof(path));
        FILE *file = fopen(path, "rb");
        imu_spin_log_header_t header;
        if (file == NULL || read_header(file, &header) != ESP_OK) {
            printf("  %03u: invalid or unreadable\n", session_id);
            if (file != NULL) {
                fclose(file);
            }
            continue;
        }

        float peak_dps = 0.0f;
        imu_spin_log_sample_t last_sample = { 0 };
        bool readable = true;
        for (size_t i = 0; i < header.sample_count; ++i) {
            imu_spin_log_sample_t sample;
            if (!read_sample(file, &header, &sample)) {
                readable = false;
                break;
            }
            if (sample.turn_rate_dps < -peak_dps || sample.turn_rate_dps > peak_dps) {
                peak_dps = sample.turn_rate_dps < 0.0f ? -sample.turn_rate_dps : sample.turn_rate_dps;
            }
            last_sample = sample;
        }
        fclose(file);
        if (!readable) {
            printf("  %03u: invalid or unreadable\n", session_id);
            continue;
        }
        printf("  %03u: %u, %lu, %d, %d, %.1f, %.1f\n", session_id,
               header.sample_count, (unsigned long)last_sample.elapsed_ms, header.left_speed,
               header.right_speed, (double)last_sample.turned_degrees, (double)peak_dps);
        ++listed;
    }
    closedir(directory);
    if (listed == 0) {
        printf("  (none)\n");
    }
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t imu_spin_log_dump(unsigned int session_id)
{
    if (s_mutex == NULL || session_id > IMU_SPIN_LOG_MAX_SESSION_ID) {
        return ESP_ERR_INVALID_ARG;
    }

    char path[IMU_SPIN_LOG_PATH_SIZE];
    session_path(session_id, path, sizeof(path));
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        int open_errno = errno;
        xSemaphoreGive(s_mutex);
        return open_errno == ENOENT ? ESP_ERR_NOT_FOUND : ESP_FAIL;
    }

    imu_spin_log_header_t header;
    esp_err_t err = read_header(file, &header);
    if (err == ESP_OK) {
        if (header.version == IMU_SPIN_LOG_VERSION_LEGACY) {
            printf("elapsed_ms,turn_rate_dps,turned_degrees,gyro_x_raw,gyro_y_raw,gyro_z_raw,temperature_mC\n");
        } else {
            printf("elapsed_ms,turn_rate_dps,turned_degrees,gyro_x_raw,gyro_y_raw,gyro_z_raw,temperature_mC,"
                   "left_target_cps,right_target_cps,left_measured_cps,right_measured_cps,"
                   "left_pwm_percent,right_pwm_percent\n");
        }
        for (size_t i = 0; i < header.sample_count; ++i) {
            imu_spin_log_sample_t sample;
            if (!read_sample(file, &header, &sample)) {
                err = ESP_FAIL;
                break;
            }
            if (header.version == IMU_SPIN_LOG_VERSION_LEGACY) {
                printf("%lu,%.3f,%.3f,%d,%d,%d,%ld\n", (unsigned long)sample.elapsed_ms,
                       (double)sample.turn_rate_dps, (double)sample.turned_degrees,
                       sample.gyro_x, sample.gyro_y, sample.gyro_z,
                       (long)sample.temperature_mC);
            } else {
                printf("%lu,%.3f,%.3f,%d,%d,%d,%ld,%d,%d,%d,%d,%d,%d\n",
                       (unsigned long)sample.elapsed_ms, (double)sample.turn_rate_dps,
                       (double)sample.turned_degrees, sample.gyro_x, sample.gyro_y,
                       sample.gyro_z, (long)sample.temperature_mC, sample.left_target_cps,
                       sample.right_target_cps, sample.left_measured_cps,
                       sample.right_measured_cps, sample.left_pwm_percent,
                       sample.right_pwm_percent);
            }
        }
    }
    fclose(file);
    xSemaphoreGive(s_mutex);
    return err;
}

esp_err_t imu_spin_log_delete(unsigned int session_id)
{
    if (s_mutex == NULL || session_id > IMU_SPIN_LOG_MAX_SESSION_ID) {
        return ESP_ERR_INVALID_ARG;
    }

    char path[IMU_SPIN_LOG_PATH_SIZE];
    session_path(session_id, path, sizeof(path));
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int result = unlink(path);
    int unlink_errno = errno;
    xSemaphoreGive(s_mutex);
    if (result == 0) {
        return ESP_OK;
    }
    return unlink_errno == ENOENT ? ESP_ERR_NOT_FOUND : ESP_FAIL;
}

esp_err_t imu_spin_log_clear(void)
{
    if (s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_recording) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    DIR *directory = opendir(IMU_SPIN_LOG_BASE_PATH);
    if (directory == NULL) {
        xSemaphoreGive(s_mutex);
        return ESP_FAIL;
    }
    esp_err_t err = ESP_OK;
    struct dirent *entry;
    while ((entry = readdir(directory)) != NULL) {
        unsigned int session_id;
        if (!parse_session_id(entry->d_name, &session_id)) {
            continue;
        }
        char path[IMU_SPIN_LOG_PATH_SIZE];
        session_path(session_id, path, sizeof(path));
        if (unlink(path) != 0) {
            err = ESP_FAIL;
        }
    }
    closedir(directory);
    xSemaphoreGive(s_mutex);
    return err;
}

void imu_spin_log_print_status(void)
{
    if (s_mutex == NULL) {
        printf("IMU spin log: unavailable\n");
        return;
    }
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    printf("IMU spin log: %s, %u/%u samples%s\n", s_recording ? "recording" : "stopped",
           s_session.header.sample_count, IMU_SPIN_LOG_MAX_SAMPLES,
           s_full ? "; buffer full" : "");
    xSemaphoreGive(s_mutex);
}
