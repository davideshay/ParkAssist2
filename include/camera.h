#ifndef CAMERA_H
#define CAMERA_H

#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "esp_camera.h"
#include "camera_pins.h"
#include "parkassist.h"

void initCamera();

#endif