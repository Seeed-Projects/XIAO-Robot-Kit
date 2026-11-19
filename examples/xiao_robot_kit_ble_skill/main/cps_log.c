#include "cps_log.h"

#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define CPS_LOG_MAGIC 0x4350534cU /* "CPSL" */
#define CPS_LOG_VERSION 1
#define CPS_LOG_BASE_PATH "/spiffs"
#define CPS_LOG_PARTITION_LABEL "cpslog"
#define CPS_LOG_PATH_SIZE 40
#define CPS_LOG_SUMMARY_SETTLE_MS 2000

static const char *TAG = "cps_log";

typedef struct __attribute__((packed)) {
    uint32_t elapsed_ms;
    int8_t set_left_speed;
    int8_t set_right_speed;
    int16_t target_left_cps;
    int16_t target_right_cps;
    int16_t measured_left_cps;
    int16_t measured_right_cps;
    int8_t left_pwm_percent;
    int8_t right_pwm_percent;
} cps_log_sample_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t sample_count;
} cps_log_header_t;

typedef struct {
    cps_log_header_t header;
    cps_log_sample_t samples[CPS_LOG_MAX_SAMPLES];
} cps_log_session_t;

_Static_assert(sizeof(cps_log_sample_t) == 16, "unexpected CPS log sample size");
_Static_assert(sizeof(cps_log_session_t) == 19208, "unexpected CPS log RAM footprint");

/*
 * This fixed 19,208-byte buffer is the hard logging limit. Only the active
 * 30-second session lives in RAM; completed sessions are separate SPIFFS files.
 */
static cps_log_session_t session;
static SemaphoreHandle_t log_mutex;
static int64_t start_time_us;
static bool recording;
static bool full;
static bool storage_ready;

typedef struct {
    int set_left_speed;
    int set_right_speed;
    uint32_t start_ms;
    uint32_t end_ms;
    size_t sample_count;
    size_t stable_sample_count;
    int64_t left_pwm_sum;
    int64_t right_pwm_sum;
    int64_t left_cps_sum;
    int64_t right_cps_sum;
} cps_log_segment_summary_t;

static int8_t clamp_i8(int value)
{
    if (value < INT8_MIN) {
        return INT8_MIN;
    }
    if (value > INT8_MAX) {
        return INT8_MAX;
    }
    return (int8_t)value;
}

static int16_t clamp_i16(int value)
{
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    return (int16_t)value;
}

static size_t session_length(const cps_log_header_t *header)
{
    return sizeof(*header) + (size_t)header->sample_count * sizeof(cps_log_sample_t);
}

static bool session_header_is_valid(const cps_log_header_t *header, size_t length)
{
    return header->magic == CPS_LOG_MAGIC && header->version == CPS_LOG_VERSION &&
           header->sample_count > 0 && header->sample_count <= CPS_LOG_MAX_SAMPLES &&
           length == session_length(header);
}

static void session_path(unsigned int session_id, char *path, size_t path_size)
{
    snprintf(path, path_size, CPS_LOG_BASE_PATH "/log_%03u.bin", session_id);
}

static bool parse_session_id(const char *name, unsigned int *session_id)
{
    char extra;
    unsigned int parsed_id;
    if (sscanf(name, "log_%u.bin%c", &parsed_id, &extra) != 1 ||
        parsed_id > CPS_LOG_MAX_SESSION_ID) {
        return false;
    }
    *session_id = parsed_id;
    return true;
}

static esp_err_t find_free_session_id(unsigned int *session_id)
{
    char path[CPS_LOG_PATH_SIZE];
    struct stat file_info;

    for (unsigned int id = 0; id <= CPS_LOG_MAX_SESSION_ID; ++id) {
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

static esp_err_t read_header(FILE *file, cps_log_header_t *header)
{
    if (fseek(file, 0, SEEK_END) != 0) {
        return ESP_FAIL;
    }
    long length = ftell(file);
    if (length < 0 || fseek(file, 0, SEEK_SET) != 0 ||
        fread(header, sizeof(*header), 1, file) != 1) {
        return ESP_FAIL;
    }
    return session_header_is_valid(header, (size_t)length) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t save_session_locked(unsigned int *session_id)
{
    unsigned int new_session_id;
    esp_err_t err = find_free_session_id(&new_session_id);
    if (err != ESP_OK) {
        return err;
    }

    char path[CPS_LOG_PATH_SIZE];
    char temporary_path[CPS_LOG_PATH_SIZE];
    session_path(new_session_id, path, sizeof(path));
    snprintf(temporary_path, sizeof(temporary_path), CPS_LOG_BASE_PATH "/log_%03u.tmp",
             new_session_id);

    FILE *file = fopen(temporary_path, "wb");
    if (file == NULL) {
        return ESP_FAIL;
    }

    size_t length = session_length(&session.header);
    size_t written = fwrite(&session, 1, length, file);
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

static esp_err_t clear_sessions_locked(void)
{
    DIR *directory = opendir(CPS_LOG_BASE_PATH);
    if (directory == NULL) {
        return ESP_FAIL;
    }

    esp_err_t err = ESP_OK;
    struct dirent *entry;
    while ((entry = readdir(directory)) != NULL) {
        unsigned int session_id;
        if (!parse_session_id(entry->d_name, &session_id)) {
            continue;
        }

        char path[CPS_LOG_PATH_SIZE];
        session_path(session_id, path, sizeof(path));
        if (unlink(path) != 0) {
            err = ESP_FAIL;
        }
    }
    closedir(directory);
    return err;
}

esp_err_t cps_log_init(void)
{
    if (log_mutex != NULL) {
        return ESP_OK;
    }

    log_mutex = xSemaphoreCreateMutex();
    if (log_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const esp_vfs_spiffs_conf_t storage_config = {
        .base_path = CPS_LOG_BASE_PATH,
        .partition_label = CPS_LOG_PARTITION_LABEL,
        .max_files = 4,
        /* A newly flashed SPIFFS partition is blank and must be formatted once. */
        .format_if_mount_failed = true,
    };
    esp_err_t err = esp_vfs_spiffs_register(&storage_config);
    if (err != ESP_OK) {
        vSemaphoreDelete(log_mutex);
        log_mutex = NULL;
        return err;
    }

    storage_ready = true;
    size_t total_bytes;
    size_t used_bytes;
    if (esp_spiffs_info(CPS_LOG_PARTITION_LABEL, &total_bytes, &used_bytes) == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS log storage ready: %u/%u bytes used", (unsigned)used_bytes,
                 (unsigned)total_bytes);
    }
    return ESP_OK;
}

esp_err_t cps_log_start(void)
{
    if (log_mutex == NULL || !storage_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(log_mutex, portMAX_DELAY);
    memset(&session.header, 0, sizeof(session.header));
    session.header.magic = CPS_LOG_MAGIC;
    session.header.version = CPS_LOG_VERSION;
    start_time_us = esp_timer_get_time();
    recording = true;
    full = false;
    xSemaphoreGive(log_mutex);
    return ESP_OK;
}

void cps_log_append(int64_t timestamp_us, int set_left_speed, int set_right_speed,
                    int target_left_cps, int target_right_cps,
                    int measured_left_cps, int measured_right_cps,
                    int left_pwm_percent, int right_pwm_percent)
{
    if (log_mutex == NULL) {
        return;
    }

    xSemaphoreTake(log_mutex, portMAX_DELAY);
    if (!recording) {
        xSemaphoreGive(log_mutex);
        return;
    }

    if (session.header.sample_count >= CPS_LOG_MAX_SAMPLES) {
        recording = false;
        full = true;
        ESP_LOGW(TAG, "RAM buffer full; run 'cpslog stop' to save %u samples",
                 session.header.sample_count);
        xSemaphoreGive(log_mutex);
        return;
    }

    cps_log_sample_t *sample = &session.samples[session.header.sample_count++];
    sample->elapsed_ms = (uint32_t)((timestamp_us - start_time_us) / 1000);
    sample->set_left_speed = clamp_i8(set_left_speed);
    sample->set_right_speed = clamp_i8(set_right_speed);
    sample->target_left_cps = clamp_i16(target_left_cps);
    sample->target_right_cps = clamp_i16(target_right_cps);
    sample->measured_left_cps = clamp_i16(measured_left_cps);
    sample->measured_right_cps = clamp_i16(measured_right_cps);
    sample->left_pwm_percent = clamp_i8(left_pwm_percent);
    sample->right_pwm_percent = clamp_i8(right_pwm_percent);

    if (session.header.sample_count == CPS_LOG_MAX_SAMPLES) {
        recording = false;
        full = true;
        ESP_LOGW(TAG, "RAM buffer full; run 'cpslog stop' to save %u samples",
                 session.header.sample_count);
    }
    xSemaphoreGive(log_mutex);
}

esp_err_t cps_log_stop_and_save(size_t *sample_count, unsigned int *session_id)
{
    if (log_mutex == NULL || sample_count == NULL || session_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(log_mutex, portMAX_DELAY);
    if (session.header.sample_count == 0) {
        xSemaphoreGive(log_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    recording = false;
    esp_err_t err = save_session_locked(session_id);
    if (err == ESP_OK) {
        *sample_count = session.header.sample_count;
        full = false;
    }
    xSemaphoreGive(log_mutex);
    return err;
}

esp_err_t cps_log_list(void)
{
    if (log_mutex == NULL || !storage_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(log_mutex, portMAX_DELAY);
    DIR *directory = opendir(CPS_LOG_BASE_PATH);
    if (directory == NULL) {
        xSemaphoreGive(log_mutex);
        return ESP_FAIL;
    }

    printf("Saved CPS logs (id, samples, duration_ms, set_left, set_right):\n");
    size_t listed = 0;
    struct dirent *entry;
    while ((entry = readdir(directory)) != NULL) {
        unsigned int session_id;
        if (!parse_session_id(entry->d_name, &session_id)) {
            continue;
        }

        char path[CPS_LOG_PATH_SIZE];
        session_path(session_id, path, sizeof(path));
        FILE *file = fopen(path, "rb");
        cps_log_header_t header;
        cps_log_sample_t first_sample;
        if (file == NULL || read_header(file, &header) != ESP_OK ||
            fread(&first_sample, sizeof(first_sample), 1, file) != 1) {
            printf("  %03u: invalid or unreadable\n", session_id);
            if (file != NULL) {
                fclose(file);
            }
            continue;
        }
        fclose(file);

        printf("  %03u: %u, %lu, %d, %d\n", session_id, header.sample_count,
               (unsigned long)(header.sample_count - 1) * CPS_LOG_SAMPLE_PERIOD_MS,
               first_sample.set_left_speed, first_sample.set_right_speed);
        ++listed;
    }
    closedir(directory);
    if (listed == 0) {
        printf("  (none)\n");
    }
    xSemaphoreGive(log_mutex);
    return ESP_OK;
}

static void print_segment_summary(unsigned int session_id,
                                  const cps_log_segment_summary_t *segment)
{
    uint32_t duration_ms = segment->end_ms - segment->start_ms;
    if (segment->stable_sample_count == 0) {
        printf("  %03u, %d, %d, -, -, %lu, 0, insufficient stable samples\n", session_id,
               segment->set_left_speed, segment->set_right_speed,
               (unsigned long)duration_ms);
        return;
    }

    int64_t left_cps = segment->left_cps_sum / (int64_t)segment->stable_sample_count;
    int64_t right_cps = segment->right_cps_sum / (int64_t)segment->stable_sample_count;
    int64_t left_pwm = segment->left_pwm_sum / (int64_t)segment->stable_sample_count;
    int64_t right_pwm = segment->right_pwm_sum / (int64_t)segment->stable_sample_count;
    printf("  %03u, %d, %d, %lld, %lld, %lu, %u, %lld, %lld, %+lld\n", session_id,
           segment->set_left_speed, segment->set_right_speed, (long long)left_pwm,
           (long long)right_pwm, (unsigned long)duration_ms,
           (unsigned)segment->stable_sample_count, (long long)left_cps,
           (long long)right_cps, (long long)(left_cps - right_cps));
}

static esp_err_t summarize_file(unsigned int session_id)
{
    char path[CPS_LOG_PATH_SIZE];
    session_path(session_id, path, sizeof(path));
    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        return errno == ENOENT ? ESP_ERR_NOT_FOUND : ESP_FAIL;
    }

    cps_log_header_t header;
    esp_err_t err = read_header(file, &header);
    if (err != ESP_OK) {
        fclose(file);
        return err;
    }

    bool found_segment = false;
    cps_log_segment_summary_t segment = { 0 };
    for (size_t i = 0; i < header.sample_count; ++i) {
        cps_log_sample_t sample;
        if (fread(&sample, sizeof(sample), 1, file) != 1) {
            err = ESP_FAIL;
            break;
        }

        bool motion_commanded = sample.set_left_speed != 0 || sample.set_right_speed != 0;
        bool command_changed = segment.sample_count != 0 &&
                               (sample.set_left_speed != segment.set_left_speed ||
                                sample.set_right_speed != segment.set_right_speed);
        if (!motion_commanded || command_changed) {
            if (segment.sample_count != 0) {
                print_segment_summary(session_id, &segment);
                found_segment = true;
                memset(&segment, 0, sizeof(segment));
            }
            if (!motion_commanded) {
                continue;
            }
        }

        if (segment.sample_count == 0) {
            segment.set_left_speed = sample.set_left_speed;
            segment.set_right_speed = sample.set_right_speed;
            segment.start_ms = sample.elapsed_ms;
        }
        segment.end_ms = sample.elapsed_ms;
        ++segment.sample_count;
        if (sample.elapsed_ms - segment.start_ms >= CPS_LOG_SUMMARY_SETTLE_MS) {
            segment.left_pwm_sum += sample.left_pwm_percent;
            segment.right_pwm_sum += sample.right_pwm_percent;
            segment.left_cps_sum += sample.measured_left_cps;
            segment.right_cps_sum += sample.measured_right_cps;
            ++segment.stable_sample_count;
        }
    }
    if (err == ESP_OK && segment.sample_count != 0) {
        print_segment_summary(session_id, &segment);
        found_segment = true;
    }
    fclose(file);

    if (err == ESP_OK && !found_segment) {
        printf("  %03u, no nonzero PWM segment\n", session_id);
    }
    return err;
}

esp_err_t cps_log_summary(void)
{
    if (log_mutex == NULL || !storage_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(log_mutex, portMAX_DELAY);
    if (recording) {
        xSemaphoreGive(log_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    DIR *directory = opendir(CPS_LOG_BASE_PATH);
    if (directory == NULL) {
        xSemaphoreGive(log_mutex);
        return ESP_FAIL;
    }

    printf("CPS summary (first %u ms of each PWM segment excluded):\n",
           (unsigned)CPS_LOG_SUMMARY_SETTLE_MS);
    printf("  id, set_left, set_right, pwm_left, pwm_right, duration_ms, stable_samples, left_cps, right_cps, diff\n");
    esp_err_t err = ESP_OK;
    struct dirent *entry;
    while ((entry = readdir(directory)) != NULL) {
        unsigned int session_id;
        if (!parse_session_id(entry->d_name, &session_id)) {
            continue;
        }
        esp_err_t file_err = summarize_file(session_id);
        if (file_err != ESP_OK) {
            printf("  %03u, invalid or unreadable\n", session_id);
            err = file_err;
        }
    }
    closedir(directory);
    xSemaphoreGive(log_mutex);
    return err;
}

esp_err_t cps_log_clear(void)
{
    if (log_mutex == NULL || !storage_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(log_mutex, portMAX_DELAY);
    recording = false;
    full = false;
    memset(&session.header, 0, sizeof(session.header));
    esp_err_t err = clear_sessions_locked();
    xSemaphoreGive(log_mutex);
    return err;
}

esp_err_t cps_log_delete(unsigned int session_id)
{
    if (log_mutex == NULL || !storage_ready || session_id > CPS_LOG_MAX_SESSION_ID) {
        return ESP_ERR_INVALID_ARG;
    }

    char path[CPS_LOG_PATH_SIZE];
    session_path(session_id, path, sizeof(path));

    xSemaphoreTake(log_mutex, portMAX_DELAY);
    int result = unlink(path);
    int unlink_errno = errno;
    xSemaphoreGive(log_mutex);
    if (result == 0) {
        return ESP_OK;
    }
    return unlink_errno == ENOENT ? ESP_ERR_NOT_FOUND : ESP_FAIL;
}

esp_err_t cps_log_dump(unsigned int session_id)
{
    if (log_mutex == NULL || !storage_ready || session_id > CPS_LOG_MAX_SESSION_ID) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(log_mutex, portMAX_DELAY);
    if (recording) {
        xSemaphoreGive(log_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    char path[CPS_LOG_PATH_SIZE];
    session_path(session_id, path, sizeof(path));
    FILE *file = fopen(path, "rb");
    if (file == NULL) {
        int open_errno = errno;
        xSemaphoreGive(log_mutex);
        return open_errno == ENOENT ? ESP_ERR_NOT_FOUND : ESP_FAIL;
    }

    cps_log_header_t header;
    esp_err_t err = read_header(file, &header);
    if (err == ESP_OK) {
        printf("elapsed_ms,set_left,set_right,target_left_cps,target_right_cps,"
               "measured_left_cps,measured_right_cps,left_pwm,right_pwm\n");
        for (size_t i = 0; i < header.sample_count; ++i) {
            cps_log_sample_t sample;
            if (fread(&sample, sizeof(sample), 1, file) != 1) {
                err = ESP_FAIL;
                break;
            }
            printf("%lu,%d,%d,%d,%d,%d,%d,%d,%d\n", (unsigned long)sample.elapsed_ms,
                   sample.set_left_speed, sample.set_right_speed,
                   sample.target_left_cps, sample.target_right_cps,
                   sample.measured_left_cps, sample.measured_right_cps,
                   sample.left_pwm_percent, sample.right_pwm_percent);
        }
    }
    fclose(file);
    xSemaphoreGive(log_mutex);
    return err;
}

void cps_log_print_status(void)
{
    if (log_mutex == NULL || !storage_ready) {
        printf("CPS log: storage not initialized\n");
        return;
    }

    xSemaphoreTake(log_mutex, portMAX_DELAY);
    size_t total_bytes = 0;
    size_t used_bytes = 0;
    esp_err_t storage_err = esp_spiffs_info(CPS_LOG_PARTITION_LABEL, &total_bytes, &used_bytes);
    printf("CPS log: %s, %u/%u samples (%u ms maximum)%s\n",
           recording ? "recording" : "stopped", session.header.sample_count,
           CPS_LOG_MAX_SAMPLES, CPS_LOG_MAX_SAMPLES * CPS_LOG_SAMPLE_PERIOD_MS,
           full ? "; buffer full, save pending" : "");
    if (storage_err == ESP_OK) {
        printf("CPS log storage: %u/%u bytes used\n", (unsigned)used_bytes,
               (unsigned)total_bytes);
    }
    xSemaphoreGive(log_mutex);
}
