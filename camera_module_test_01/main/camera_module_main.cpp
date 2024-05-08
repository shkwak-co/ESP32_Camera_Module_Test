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


static QueueHandle_t xQueueCameraFrame = NULL;

extern "C" void
app_main (void)
{
  app_wifi_main ();
  xQueueCameraFrame = xQueueCreate (2, sizeof (camera_fb_t *));
  register_camera (PIXFORMAT_RGB565, FRAMESIZE_QVGA, 2, xQueueCameraFrame);
  app_mdns_main ();
  register_httpd (xQueueCameraFrame, NULL, true);
}
