#include <string.h>
#include <stdio.h>

// who_camera.h
#include "who_camera.h"
#include "freertos/FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_camera.h"

// app_wifi
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/ip4_addr.h"
#include "freertos/event_groups.h"
#include "mdns.h"

// app_httpd
#include "app_httpd.hpp"
#include <list>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "app_mdns.h"
#include "sdkconfig.h"
#include "app_wifi.h"

// http_server
#include <stdlib.h>
#include <unistd.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_tls.h"

#include "esp_err.h"
#include "esp_wifi_types.h"

static const char *TAG = "UNIT_TEST";

// Prototype BEGIN
static size_t jpg_encode_stream (void *arg, size_t index, const void *data, size_t len);
static esp_err_t capture_handler (httpd_req_t * req);
static esp_err_t stream_handler (httpd_req_t * req);
static esp_err_t index_handler (httpd_req_t * req);
void open_httpd (const QueueHandle_t frame_i, const QueueHandle_t frame_o, const bool return_fb);



// Prototype END



/**
 * @brief
 * - PART_BOUNDARY는 HTTP
 * 프로토콜의 바디 부분에 데이터를 여러 부분으로 나눠서 보내는
 * 멀티 파트 스트림에서 경계를 나타냄.
 * 
 * - _STREAM_CONTENT_TYPE
 * - _STREAM_BOUNDARY
 * HTTP 응답에서 type과 PART_BOUNDARY 설정
 * 
 * - _STREAM_PART
 * Timestamp와 content-type을 image/jpeg로 설정
 * 
*/
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";


static QueueHandle_t xQueueCameraFrame = NULL;
static QueueHandle_t xQueueCameraO = NULL;

/**
 * httpd
 * 
 * - xQueueFrameI
 * 카메라로부터 받는 프레임 버퍼
 * 
 * - xQueueFrameO
 * 서버로 전송하는 프레임 버퍼
 * 
 * - gReturnFB
 * 프레임 버퍼가 사용 가능한지 확인용
 * 
*/
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;
static QueueHandle_t xQueueFrameI = NULL;
static QueueHandle_t xQueueFrameO = NULL;
static bool gReturnFB = true;

// server open BEGIN

typedef struct
{
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

static size_t
jpg_encode_stream (void *arg, size_t index, const void *data, size_t len)
{
  jpg_chunking_t *j = (jpg_chunking_t *) arg;
  if (!index)
    {
      j->len = 0;
    }
  if (httpd_resp_send_chunk (j->req, (const char *) data, len) != ESP_OK)
    {
      return 0;
    }
  j->len += len;
  return len;
}


static esp_err_t
capture_handler (httpd_req_t * req)
{
  camera_fb_t *frame = NULL;
  esp_err_t res = ESP_OK;

  if (xQueueReceive (xQueueFrameI, &frame, portMAX_DELAY))
    {
      httpd_resp_set_type (req, "image/jpeg");
      httpd_resp_set_hdr (req, "Content-Disposition", "inline; filename=capture.jpg");
      httpd_resp_set_hdr (req, "Access-Control-Allow-Origin", "*");

      char ts[32];
      snprintf (ts, 32, "%lld.%06ld", frame->timestamp.tv_sec, frame->timestamp.tv_usec);
      httpd_resp_set_hdr (req, "X-Timestamp", (const char *) ts);

      // size_t fb_len = 0;
      if (frame->format == PIXFORMAT_JPEG)
        {
          // fb_len = frame->len;
          res = httpd_resp_send (req, (const char *) frame->buf, frame->len);
        }
      else
        {
          jpg_chunking_t jchunk = { req, 0 };
          res = frame2jpg_cb (frame, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
          httpd_resp_send_chunk (req, NULL, 0);
          // fb_len = jchunk.len;
        }

      if (xQueueFrameO)
        {
          xQueueSend (xQueueFrameO, &frame, portMAX_DELAY);
        }
      else if (gReturnFB)
        {
          esp_camera_fb_return (frame);
        }
      else
        {
          free (frame);
        }
    }
  else
    {
      ESP_LOGE (TAG, "Camera capture failed");
      httpd_resp_send_500 (req);
      return ESP_FAIL;
    }

  return res;
}

<<<<<<< Updated upstream
=======
httpd_uri_t pic_uri = {
  .uri = "/capture",
  .method = HTTP_GET,
  .handler = pic_handler,
  .user_ctx = NULL
};


>>>>>>> Stashed changes
static esp_err_t
stream_handler (httpd_req_t * req)
{
  camera_fb_t *frame = NULL;
  struct timeval _timestamp;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[128];

  res = httpd_resp_set_type (req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK)
    {
      return res;
    }

  httpd_resp_set_hdr (req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr (req, "X-Framerate", "60");

  while (true)
    {
      if (xQueueReceive (xQueueFrameI, &frame, portMAX_DELAY))
        {
          _timestamp.tv_sec = frame->timestamp.tv_sec;
          _timestamp.tv_usec = frame->timestamp.tv_usec;

          if (frame->format == PIXFORMAT_JPEG)
            {
              _jpg_buf = frame->buf;
              _jpg_buf_len = frame->len;
            }
          else if (!frame2jpg (frame, 80, &_jpg_buf, &_jpg_buf_len))
            {
              ESP_LOGE (TAG, "JPEG compression failed");
              res = ESP_FAIL;
            }
        }
      else
        {
          res = ESP_FAIL;
        }

      if (res == ESP_OK)
        {
          res = httpd_resp_send_chunk (req, _STREAM_BOUNDARY, strlen (_STREAM_BOUNDARY));
        }

      if (res == ESP_OK)
        {
          size_t hlen =
            snprintf ((char *) part_buf, 128, _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
          res = httpd_resp_send_chunk (req, (const char *) part_buf, hlen);
        }

      if (res == ESP_OK)
        {
          res = httpd_resp_send_chunk (req, (const char *) _jpg_buf, _jpg_buf_len);
        }

      if (frame->format != PIXFORMAT_JPEG)
        {
          free (_jpg_buf);
          _jpg_buf = NULL;
        }

      if (xQueueFrameO)
        {
          xQueueSend (xQueueFrameO, &frame, portMAX_DELAY);
        }
      else if (gReturnFB)
        {
          esp_camera_fb_return (frame);
        }
      else
        {
          free (frame);
        }

      if (res != ESP_OK)
        {
          break;
        }
    }

  return res;
}

/**
 * index_handler
 * _binary_..._html_gz 파일을 읽어서 웹 서버을 구성
 * 
 * 사용중인 EYE보드는 OV2640 카메라 모듈을 사용하므로 
 * 다른 모델에 해당하는 코드는 삭제했음.
*/
static esp_err_t
index_handler (httpd_req_t * req)
{
  extern const unsigned char index_ov2640_html_gz_start[] asm ("_binary_index_ov2640_html_gz_start");
  extern const unsigned char index_ov2640_html_gz_end[] asm ("_binary_index_ov2640_html_gz_end");
  size_t index_ov2640_html_gz_len = index_ov2640_html_gz_end - index_ov2640_html_gz_start;

  httpd_resp_set_type (req, "text/html");
  httpd_resp_set_hdr (req, "Content-Encoding", "gzip");
  sensor_t *s = esp_camera_sensor_get ();
  if (s != NULL)
    {
      if (s->id.PID == OV2640_PID)
        {
          return httpd_resp_send (req, (const char *) index_ov2640_html_gz_start, index_ov2640_html_gz_len);
        }
    }
  else
    {
      ESP_LOGE (TAG, "Camera sensor not found");

    }
  return httpd_resp_send_500 (req);
}

void
open_httpd (const QueueHandle_t frame_i, const QueueHandle_t frame_o, const bool return_fb)
{
  xQueueFrameI = frame_i;
  xQueueFrameO = frame_o;
  gReturnFB = return_fb;

  httpd_config_t config = HTTPD_DEFAULT_CONFIG ();


  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
  };

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  httpd_uri_t capture_uri = {
    .uri = "/capture",
    .method = HTTP_GET,
    .handler = capture_handler,
    .user_ctx = NULL
  };


  ESP_LOGI (TAG, "Starting web server on port: '%d'", config.server_port);
  if (httpd_start (&camera_httpd, &config) == ESP_OK)
    {
      httpd_register_uri_handler (camera_httpd, &index_uri);
      httpd_register_uri_handler (camera_httpd, &capture_uri);
    }

  config.server_port += 1;
  config.ctrl_port += 1;
  ESP_LOGI (TAG, "Starting stream server on port: '%d'", config.server_port);
  if (httpd_start (&stream_httpd, &config) == ESP_OK)
    {
      httpd_register_uri_handler (stream_httpd, &stream_uri);
    }
}

// server open END

// camera init BEGIN
static void task_process_handler(void *arg)
{
    while (true)
    {
        camera_fb_t *frame = esp_camera_fb_get();
        if (frame)
            xQueueSend(xQueueCameraO, &frame, portMAX_DELAY);
    }
}

void camera_settings(const pixformat_t pixel_fromat,
                     const framesize_t frame_size,
                     const uint8_t fb_count,
                     const QueueHandle_t frame_o)
{
    ESP_LOGI(TAG, "Camera module is %s", CAMERA_MODULE_NAME);

#if CONFIG_CAMERA_MODULE_ESP_EYE || CONFIG_CAMERA_MODULE_ESP32_CAM_BOARD
    /* IO13, IO14 is designed for JTAG by default,
     * to use it as generalized input,
     * firstly declair it as pullup input */
    gpio_config_t conf;
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_up_en = GPIO_PULLUP_ENABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    conf.pin_bit_mask = 1LL << 13;
    gpio_config(&conf);
    conf.pin_bit_mask = 1LL << 14;
    gpio_config(&conf);
#endif

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAMERA_PIN_D0;
    config.pin_d1 = CAMERA_PIN_D1;
    config.pin_d2 = CAMERA_PIN_D2;
    config.pin_d3 = CAMERA_PIN_D3;
    config.pin_d4 = CAMERA_PIN_D4;
    config.pin_d5 = CAMERA_PIN_D5;
    config.pin_d6 = CAMERA_PIN_D6;
    config.pin_d7 = CAMERA_PIN_D7;
    config.pin_xclk = CAMERA_PIN_XCLK;
    config.pin_pclk = CAMERA_PIN_PCLK;
    config.pin_vsync = CAMERA_PIN_VSYNC;
    config.pin_href = CAMERA_PIN_HREF;
    config.pin_sscb_sda = CAMERA_PIN_SIOD;
    config.pin_sscb_scl = CAMERA_PIN_SIOC;
    config.pin_pwdn = CAMERA_PIN_PWDN;
    config.pin_reset = CAMERA_PIN_RESET;
    config.xclk_freq_hz = XCLK_FREQ_HZ;
    config.pixel_format = pixel_fromat;
    config.frame_size = frame_size;
    config.jpeg_quality = 12;
    config.fb_count = fb_count;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    //config.grab_mode = CAMERA_GRAB_LATEST; //차이를 잘 모르겠음

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID || s->id.PID == OV2640_PID) {
        s->set_vflip(s, 1); //flip it back    
    } else if (s->id.PID == GC0308_PID) {
        s->set_hmirror(s, 0);
    } else if (s->id.PID == GC032A_PID) {
        s->set_vflip(s, 1);
    }

    //initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID)
    {
        s->set_brightness(s, 1);  //up the blightness just a bit
        s->set_saturation(s, -2); //lower the saturation
    }

    xQueueCameraO = frame_o;
    xTaskCreatePinnedToCore(task_process_handler, TAG, 3 * 1024, NULL, 5, NULL, 1);
}


// camera init END

extern "C" void
app_main (void)
{
<<<<<<< Updated upstream
=======
  // 기존 AP모드 설정과 동일하여 그냥 사용함
>>>>>>> Stashed changes
  app_wifi_main ();
  xQueueCameraFrame = xQueueCreate (2, sizeof (camera_fb_t *));
  //camera_settings (PIXFORMAT_RGB565, FRAMESIZE_QVGA, 2, xQueueCameraFrame);
  camera_settings (PIXFORMAT_JPEG, FRAMESIZE_QVGA, 2, xQueueCameraFrame);
  app_mdns_main ();
  open_httpd (xQueueCameraFrame, NULL, true);
}
