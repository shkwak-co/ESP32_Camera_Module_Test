# ESP32 Camera Module Test

ESP32-CAM and camera-related ESP-IDF examples.

## Examples

- `camera_example/`
- `camera_module_test_01/`
- `jpeg_http_test/`
- `web/`
- `sample_project_cam/`: ESP32-CAM SoftAP web camera example.

## `sample_project_cam`

`sample_project_cam` runs an ESP32-CAM as a Wi-Fi SoftAP and serves a browser UI.

- Wi-Fi SSID: `ESP32-CAM`
- Wi-Fi password: `esp32cam`
- Web UI: `http://192.168.4.1`
- Photo endpoint: `GET /capture`
- MJPEG stream endpoint: `GET http://192.168.4.1:81/stream`

The default camera pin map is for the common AI Thinker ESP32-CAM module. If your board is different, update the `CAM_PIN_*` constants in `sample_project_cam/main/camera_app.c`.

### Build

```bash
cd sample_project_cam
idf.py build
```

### Flash and Monitor

```bash
idf.py -p <PORT> flash monitor
```

After flashing, connect your phone or PC to Wi-Fi `ESP32-CAM` and open `http://192.168.4.1`.
