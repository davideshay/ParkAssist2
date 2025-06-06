#include <Arduino.h>
#include <camera.h>

AsyncWebServer serverCam(82);

void initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_UXGA;
    config.pixel_format = PIXFORMAT_JPEG;  // for streaming
    //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  
    // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
    //                      for larger pre-allocated frame buffer.
    if (config.pixel_format == PIXFORMAT_JPEG) {
      if (psramFound()) {
        config.jpeg_quality = 10;
        config.fb_count = 2;
        config.grab_mode = CAMERA_GRAB_LATEST;
      } else {
        // Limit the frame size when PSRAM is not available
        config.frame_size = FRAMESIZE_SVGA;
        config.fb_location = CAMERA_FB_IN_DRAM;
      }
    } else {
      // Best option for face detection/recognition
      config.frame_size = FRAMESIZE_240X240;
      #if CONFIG_IDF_TARGET_ESP32S3
        config.fb_count = 2;
      #endif  
    }
      // camera init
      esp_err_t err = esp_camera_init(&config);
      if (err != ESP_OK) {
        logData("Camera init failed with error :" + err, true);
        return;
      }
      sensor_t *s = esp_camera_sensor_get();
      // drop down frame size for higher initial frame rate
      if (config.pixel_format == PIXFORMAT_JPEG) {
        s->set_framesize(s, FRAMESIZE_QVGA);
      }
      #if defined(CAMERA_MODEL_ESP32S3_EYE)
        s->set_vflip(s, 1);
      #endif
    
      serverCam.on("/picture", HTTP_GET, [](AsyncWebServerRequest * request) {
        camera_fb_t * frame = NULL;
        frame = esp_camera_fb_get();
        request->send(200, "image/jpeg", (const uint8_t *)frame->buf, frame->len);
        esp_camera_fb_return(frame);
      });
      serverCam.begin();
      logData("Camera server started",true);
    
  }  