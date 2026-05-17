#include "camera_app.h"
#include "web_server.h"
#include "wifi_app.h"

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "app_main";

static esp_err_t init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

void app_main(void)
{
    ESP_ERROR_CHECK(init_nvs());
    ESP_ERROR_CHECK(wifi_app_start_softap());

    esp_err_t camera_err = camera_app_init();
    if (camera_err != ESP_OK) {
        ESP_LOGW(TAG, "HTTP UI will start, but camera requests will return an error");
    }

    ESP_ERROR_CHECK(web_server_start());
}
