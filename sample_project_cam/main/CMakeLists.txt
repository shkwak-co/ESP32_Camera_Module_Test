idf_component_register(
    SRCS
        "main.c"
        "camera_app.c"
        "wifi_app.c"
        "web_server.c"
    INCLUDE_DIRS "."
    EMBED_TXTFILES "www/index.html"
    REQUIRES
        esp_wifi
        esp_http_server
        esp_netif
        esp_event
        nvs_flash
        esp_timer
        esp_psram
        esp_driver_gpio
)
