#include <stdio.h>

#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>

#include "esp_wifi.h"
#include "esp_http_server.h"

#include "freertos/FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_camera.h"
#include "who_camera.h"
#include "who_human_face_detection.hpp"
#include "app_httpd.hpp"
#include "app_wifi.h"
#include "app_mdns.h"


#define TAG "module_test_main"

/*
  카메라로 촬영한
  JPEG파일을 http 서버로 출력
*/

static camera_config_t camera_config = {
  .pin_pwdn = CAMERA_PIN_PWDN,
  .pin_reset = CAMERA_PIN_RESET,
  .pin_xclk = CAMERA_PIN_XCLK,
  .pin_sccb_sda = CAMERA_PIN_SIOD,
  .pin_sccb_scl = CAMERA_PIN_SIOC,

  .pin_d7 = CAMERA_PIN_D7,
  .pin_d6 = CAMERA_PIN_D6,
  .pin_d5 = CAMERA_PIN_D5,
  .pin_d4 = CAMERA_PIN_D4,
  .pin_d3 = CAMERA_PIN_D3,
  .pin_d2 = CAMERA_PIN_D2,
  .pin_d1 = CAMERA_PIN_D1,
  .pin_d0 = CAMERA_PIN_D0,
  .pin_vsync = CAMERA_PIN_VSYNC,
  .pin_href = CAMERA_PIN_HREF,
  .pin_pclk = CAMERA_PIN_PCLK,

  //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
  .xclk_freq_hz = XCLK_FREQ_HZ,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,


  /**
   * format을 JPEG로 설정하면, RGP565일 때 보다 속도가 빠르며 
   * 연속 요청시 종료되지 않음. 상황에 맞게 사용할 것
  */
  //.pixel_format = PIXFORMAT_RGB565, //YUV422,GRAYSCALE,RGB565,JPEG
  .pixel_format = PIXFORMAT_JPEG,
  .frame_size = FRAMESIZE_QVGA, //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

  .jpeg_quality = 12,           //0-63, for OV series camera sensors, lower number means higher quality
  .fb_count = 1,                //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
  .fb_location = CAMERA_FB_IN_PSRAM,
  .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

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
pic_handler (httpd_req_t * req)
{
  camera_fb_t *frame = NULL;
  esp_err_t res = ESP_OK;

  ESP_LOGI (TAG, "captured");
  frame = esp_camera_fb_get ();

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
      ESP_LOGI (TAG, "already jpg");
      res = httpd_resp_send (req, (const char *) frame->buf, frame->len);
    }
  else
    {
      ESP_LOGI (TAG, "convert to jpg");
      jpg_chunking_t jchunk = { req, 0 };
      res = frame2jpg_cb (frame, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
      httpd_resp_send_chunk (req, NULL, 0);
      // fb_len = jchunk.len;
    }

  esp_camera_fb_return (frame);
  ESP_LOGI (TAG, "done");

  return res;
}


httpd_uri_t pic_uri = {
  .uri = "/",
  .method = HTTP_GET,
  .handler = pic_handler,
  .user_ctx = NULL
};

void
open_server ()
{
  httpd_handle_t pic_httpd;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG ();

  if (httpd_start (&pic_httpd, &config) == ESP_OK)
    {
      httpd_register_uri_handler (pic_httpd, &pic_uri);
    }
}

extern "C" void
app_main (void)
{
  // 기존 AP모드 설정과 동일하여 그냥 사용함
  app_wifi_main ();

  if (esp_camera_init (&camera_config) != ESP_OK)
    {
      ESP_LOGE (TAG, "camera init failed");
      goto err;
    }
  app_mdns_main ();

  open_server ();

err:
}
