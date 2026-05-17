#include "web_server.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "camera_app.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define STREAM_HTTP_PORT 81
#define STREAM_HTTP_CTRL_PORT 32769
#define PART_BOUNDARY "123456789000000000000987654321"

static const char *TAG = "web_server";

static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");

static uint32_t s_stream_generation;

static esp_err_t send_camera_not_ready(httpd_req_t *req)
{
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "Camera is not ready. Check the ESP32-CAM pin map and PSRAM settings.");
    return ESP_OK;
}

static esp_err_t index_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "GET /");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req,
                           (const char *)index_html_start,
                           index_html_end - index_html_start);
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    if (!camera_app_is_ready()) {
        return send_camera_not_ready(req);
    }

    ESP_LOGI(TAG, "GET /capture");

    uint8_t *jpg = NULL;
    size_t jpg_len = 0;
    int64_t start = esp_timer_get_time();
    esp_err_t res = camera_app_get_latest_jpeg(&jpg, &jpg_len, pdMS_TO_TICKS(5000));
    if (res == ESP_ERR_TIMEOUT) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "Camera is busy. Try again.");
        return ESP_OK;
    }
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Camera capture failed: %s", esp_err_to_name(res));
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    res = httpd_resp_send(req, (const char *)jpg, jpg_len);

    int64_t elapsed_ms = (esp_timer_get_time() - start) / 1000;
    ESP_LOGI(TAG, "JPEG capture: %u KB in %lld ms",
             (unsigned)(jpg_len / 1024), (long long)elapsed_ms);

    camera_app_free_jpeg(jpg);
    return res;
}

static esp_err_t stop_stream_handler(httpd_req_t *req)
{
    uint32_t stream_id = __atomic_add_fetch(&s_stream_generation, 1, __ATOMIC_SEQ_CST);
    ESP_LOGI(TAG, "GET /stop_stream generation=%u", (unsigned)stream_id);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    if (!camera_app_is_ready()) {
        return send_camera_not_ready(req);
    }

    uint32_t stream_id = __atomic_add_fetch(&s_stream_generation, 1, __ATOMIC_SEQ_CST);
    ESP_LOGI(TAG, "GET /stream start id=%u", (unsigned)stream_id);

    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    char part_buf[80];
    int64_t last_frame = esp_timer_get_time();

    while (__atomic_load_n(&s_stream_generation, __ATOMIC_SEQ_CST) == stream_id) {
        uint8_t *jpg = NULL;
        size_t jpg_len = 0;
        res = camera_app_get_latest_jpeg(&jpg, &jpg_len, pdMS_TO_TICKS(2000));
        if (res != ESP_OK) {
            ESP_LOGW(TAG, "Camera stream capture skipped: %s", esp_err_to_name(res));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        size_t header_len = snprintf(part_buf, sizeof(part_buf), STREAM_PART, (unsigned)jpg_len);
        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, part_buf, header_len);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)jpg, jpg_len);
        }

        size_t frame_len = jpg_len;
        camera_app_free_jpeg(jpg);

        if (res != ESP_OK) {
            break;
        }

        int64_t now = esp_timer_get_time();
        int64_t frame_time_ms = (now - last_frame) / 1000;
        last_frame = now;
        ESP_LOGD(TAG, "MJPEG frame: %u KB in %lld ms",
                 (unsigned)(frame_len / 1024), (long long)frame_time_ms);

        vTaskDelay(pdMS_TO_TICKS(75));
    }

    ESP_LOGI(TAG, "Stream connection closed id=%u", (unsigned)stream_id);
    return res;
}

static httpd_handle_t start_web_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start web server");
        return NULL;
    }

    const httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t capture_uri = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = capture_handler,
        .user_ctx = NULL,
    };
    const httpd_uri_t stop_stream_uri = {
        .uri = "/stop_stream",
        .method = HTTP_GET,
        .handler = stop_stream_handler,
        .user_ctx = NULL,
    };

    httpd_register_uri_handler(server, &index_uri);
    httpd_register_uri_handler(server, &capture_uri);
    httpd_register_uri_handler(server, &stop_stream_uri);
    ESP_LOGI(TAG, "Web server started on port 80");
    return server;
}

static httpd_handle_t start_stream_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = STREAM_HTTP_PORT;
    config.ctrl_port = STREAM_HTTP_CTRL_PORT;
    config.lru_purge_enable = true;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start stream server");
        return NULL;
    }

    const httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL,
    };

    httpd_register_uri_handler(server, &stream_uri);
    ESP_LOGI(TAG, "Stream server started on port %d", STREAM_HTTP_PORT);
    return server;
}

esp_err_t web_server_start(void)
{
    if (!start_web_server()) {
        return ESP_FAIL;
    }

    if (!start_stream_server()) {
        return ESP_FAIL;
    }

    return ESP_OK;
}
