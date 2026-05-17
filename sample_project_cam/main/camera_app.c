#include "camera_app.h"

#include <stdlib.h>
#include <string.h>

#include "esp_camera.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/*
 * Default pin map for the common AI Thinker ESP32-CAM module.
 * If your board is different, change only these constants.
 */
#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#define CAM_XCLK_FREQ_HZ 10000000

#define CAMERA_TASK_STACK_BYTES 6144
#define CAMERA_TASK_PRIORITY 5
#define CAMERA_TASK_CORE 1
#define CAMERA_FRAME_INTERVAL_MS 100
#define CAMERA_FRAME_WAIT_SLICE_MS 50

static const char *TAG = "camera_app";

static bool s_camera_ready;
static SemaphoreHandle_t s_frame_mutex;
static TaskHandle_t s_camera_task;
static uint8_t *s_latest_jpeg;
static size_t s_latest_jpeg_len;
static uint32_t s_latest_jpeg_seq;

static bool psram_is_available(void)
{
#if CONFIG_SPIRAM
    return esp_psram_is_initialized() && esp_psram_get_size() > 0;
#else
    return false;
#endif
}

static camera_config_t make_camera_config(bool use_psram)
{
    camera_config_t config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,
        .xclk_freq_hz = CAM_XCLK_FREQ_HZ,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_JPEG,
    };

    if (use_psram) {
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 14;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    } else {
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 15;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_DRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    }

    return config;
}

static void publish_latest_jpeg(uint8_t *jpeg, size_t jpeg_len)
{
    uint8_t *old_jpeg = NULL;

    if (xSemaphoreTake(s_frame_mutex, portMAX_DELAY) == pdTRUE) {
        old_jpeg = s_latest_jpeg;
        s_latest_jpeg = jpeg;
        s_latest_jpeg_len = jpeg_len;
        s_latest_jpeg_seq++;
        xSemaphoreGive(s_frame_mutex);
    } else {
        free(jpeg);
    }

    free(old_jpeg);
}

static void camera_capture_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Camera capture task started on core %d", xPortGetCoreID());
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Camera frame grab failed");
            vTaskDelay(pdMS_TO_TICKS(250));
            last_wake = xTaskGetTickCount();
            continue;
        }

        if (fb->format != PIXFORMAT_JPEG) {
            ESP_LOGE(TAG, "Unexpected frame format: %d", fb->format);
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(250));
            last_wake = xTaskGetTickCount();
            continue;
        }

        uint8_t *copy = malloc(fb->len);
        if (!copy) {
            ESP_LOGW(TAG, "Failed to allocate %u bytes for latest JPEG", (unsigned)fb->len);
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(250));
            last_wake = xTaskGetTickCount();
            continue;
        }

        memcpy(copy, fb->buf, fb->len);
        size_t copy_len = fb->len;
        esp_camera_fb_return(fb);
        publish_latest_jpeg(copy, copy_len);

        ESP_LOGD(TAG, "Published camera frame #%u: %u KB",
                 (unsigned)s_latest_jpeg_seq, (unsigned)(copy_len / 1024));
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CAMERA_FRAME_INTERVAL_MS));
    }
}

esp_err_t camera_app_init(void)
{
    if (!s_frame_mutex) {
        s_frame_mutex = xSemaphoreCreateMutex();
        if (!s_frame_mutex) {
            ESP_LOGE(TAG, "Failed to create frame mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    bool use_psram = psram_is_available();

    if (use_psram) {
        ESP_LOGI(TAG, "PSRAM detected: %u bytes", (unsigned)esp_psram_get_size());
    } else {
        ESP_LOGW(TAG, "PSRAM is not available; using QVGA and one DRAM frame buffer");
    }

    camera_config_t config = make_camera_config(use_psram);
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Camera init failed with preferred config: %s", esp_err_to_name(err));

        if (use_psram) {
            ESP_LOGW(TAG, "Retrying camera init with QVGA DRAM fallback");
            esp_camera_deinit();
            config = make_camera_config(false);
            err = esp_camera_init(&config);
        }
    }

    if (err != ESP_OK) {
        s_camera_ready = false;
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "If this is not an AI Thinker ESP32-CAM, update the CAM_PIN_* constants");
        return err;
    }

    BaseType_t task_ok = xTaskCreatePinnedToCore(camera_capture_task,
                                                 "camera_capture",
                                                 CAMERA_TASK_STACK_BYTES,
                                                 NULL,
                                                 CAMERA_TASK_PRIORITY,
                                                 &s_camera_task,
                                                 CAMERA_TASK_CORE);
    if (task_ok != pdPASS) {
        s_camera_ready = false;
        ESP_LOGE(TAG, "Failed to create camera capture task");
        esp_camera_deinit();
        return ESP_ERR_NO_MEM;
    }

    s_camera_ready = true;
    ESP_LOGI(TAG, "Camera initialized");
    return ESP_OK;
}

bool camera_app_is_ready(void)
{
    return s_camera_ready;
}

esp_err_t camera_app_get_latest_jpeg(uint8_t **out_buf, size_t *out_len, TickType_t wait_ticks)
{
    *out_buf = NULL;
    *out_len = 0;

    if (!s_camera_ready || !s_frame_mutex) {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t started = xTaskGetTickCount();

    while (true) {
        if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(CAMERA_FRAME_WAIT_SLICE_MS)) == pdTRUE) {
            if (s_latest_jpeg && s_latest_jpeg_len > 0) {
                uint8_t *copy = malloc(s_latest_jpeg_len);
                if (!copy) {
                    xSemaphoreGive(s_frame_mutex);
                    return ESP_ERR_NO_MEM;
                }

                memcpy(copy, s_latest_jpeg, s_latest_jpeg_len);
                *out_buf = copy;
                *out_len = s_latest_jpeg_len;
                xSemaphoreGive(s_frame_mutex);
                return ESP_OK;
            }

            xSemaphoreGive(s_frame_mutex);
        }

        if (wait_ticks == 0 || xTaskGetTickCount() - started >= wait_ticks) {
            return ESP_ERR_TIMEOUT;
        }

        vTaskDelay(pdMS_TO_TICKS(CAMERA_FRAME_WAIT_SLICE_MS));
    }
}

void camera_app_free_jpeg(uint8_t *buf)
{
    free(buf);
}
