#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <WiFiClient.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include <ElegantOTA.h>
#include "wificodes.h"
#include <HCSR04.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "esp_camera.h"
#include <FastLED.h>

// 
// PIN LAYOUT
// 
// Camera uses pins 4,5,6,7,15,16,17,18,8,9,10,11,12,13
//    (all of one side of the board except for 3 JTAG and 46 LOG
//
// 1 - OneWire - Temperature Sensor
// 42 - HCSR04 Trigger (out)
// 41 - HCSR04 Echo (in)
// 40 - IR Break Sensor
// 39 - WS2812 LED Panel Out
#define TEMP_SENSOR_BUS 14
#define DIST_SENSOR_TRIGGER 19
#define DIST_SENSOR_ECHO 20
#define IR_BREAK_SENSOR 21
#define LED_PANEL_PIN 47

#define COLOR_ORDER RGB
#define CHIPSET     WS2812
#define BRIGHTNESS 64

// Params for width and height
const uint8_t kMatrixWidth = 16;
const uint8_t kMatrixHeight = 16;

// Param for different pixel layouts
const bool    kMatrixSerpentineLayout = true;
const bool    kMatrixVertical = false;

#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
#include "camera_pins.h"

AsyncWebServer serverOTA(80);
AsyncWebServer serverLog(81);
AsyncWebServer serverCam(82);

UltraSonicDistanceSensor distanceSensor(DIST_SENSOR_TRIGGER, DIST_SENSOR_ECHO);  // Initialize sensor that uses digital pins 42 and 41

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(TEMP_SENSOR_BUS);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature tempSensors(&oneWire);

unsigned long ota_progress_millis = 0;

uint16_t XY( uint8_t x, uint8_t y)
{
  uint16_t i;
  
  if( kMatrixSerpentineLayout == false) {
    if (kMatrixVertical == false) {
      i = (y * kMatrixWidth) + x;
    } else {
      i = kMatrixHeight * (kMatrixWidth - (x+1))+y;
    }
  }

  if( kMatrixSerpentineLayout == true) {
    if (kMatrixVertical == false) {
      if( y & 0x01) {
        // Odd rows run backwards
        uint8_t reverseX = (kMatrixWidth - 1) - x;
        i = (y * kMatrixWidth) + reverseX;
      } else {
        // Even rows run forwards
        i = (y * kMatrixWidth) + x;
      }
    } else { // vertical positioning
      if ( x & 0x01) {
        i = kMatrixHeight * (kMatrixWidth - (x+1))+y;
      } else {
        i = kMatrixHeight * (kMatrixWidth - x) - (y+1);
      }
    }
  }
  
  return i;
}

#define NUM_LEDS (kMatrixWidth * kMatrixHeight)
CRGB leds_plus_safety_pixel[ NUM_LEDS + 1];
CRGB* const leds( leds_plus_safety_pixel + 1);

uint16_t XYsafe( uint8_t x, uint8_t y)
{
  if( x >= kMatrixWidth) return -1;
  if( y >= kMatrixHeight) return -1;
  return XY(x,y);
}

void DrawOneFrame( uint8_t startHue8, int8_t yHueDelta8, int8_t xHueDelta8)
{
  uint8_t lineStartHue = startHue8;
  for( uint8_t y = 0; y < kMatrixHeight; y++) {
    lineStartHue += yHueDelta8;
    uint8_t pixelHue = lineStartHue;      
    for( uint8_t x = 0; x < kMatrixWidth; x++) {
      pixelHue += xHueDelta8;
      leds[ XY(x, y)]  = CHSV( pixelHue, 255, 255);
    }
  }
}


void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}

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
      Serial.printf("Camera init failed with error 0x%x", err);
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
  
}  

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(IR_BREAK_SENSOR, INPUT);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
 
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
 
  serverOTA.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is ElegantOTA AsyncDemo.");
  });

  serverLog.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is WebSerial demo. You can access the WebSerial interface at http://" + WiFi.localIP().toString() + "/webserial");
  });
 
  ElegantOTA.begin(&serverOTA);    // Start ElegantOTA
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);

  serverOTA.begin();
  Serial.println("HTTP server started");

  WebSerial.begin(&serverLog);
  WebSerial.onMessage([](uint8_t *data, size_t len) {
    Serial.printf("Received %lu bytes from WebSerial: ", len);
    Serial.write(data, len);
    Serial.println();
    WebSerial.println("Received Data...");
    String d = "";
    for(size_t i = 0; i < len; i++){
      d += char(data[i]);
    }
    WebSerial.println(d);
  });

  serverLog.begin();
  tempSensors.begin();
  initCamera();

  FastLED.addLeds<CHIPSET, LED_PANEL_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalSMD5050);
  FastLED.setBrightness( BRIGHTNESS );

  serverCam.on("/picture", HTTP_GET, [](AsyncWebServerRequest * request) {
    camera_fb_t * frame = NULL;
    frame = esp_camera_fb_get();
    request->send_P(200, "image/jpeg", (const uint8_t *)frame->buf, frame->len);
    esp_camera_fb_return(frame);
  });
  serverCam.begin();

}

void loop() {
    static unsigned long last_print_time = millis();
    ElegantOTA.loop();
    WebSerial.loop();

    if ((unsigned long)(millis() - last_print_time) > 500) {
        tempSensors.requestTemperatures(); 
        float temperatureC = tempSensors.getTempCByIndex(0);
        WebSerial.print(temperatureC);
        WebSerial.println(F("degrees C"));
        double distance = distanceSensor.measureDistanceCm(temperatureC);  
        WebSerial.print(distance);
        WebSerial.println(F("cm"));
        last_print_time = millis();
        if (digitalRead(IR_BREAK_SENSOR) == LOW) {
          WebSerial.println(F("IR SENSOR LINK BROKEN - CAR PRESENT"));
        } else {
          WebSerial.println(F("IR SENSOR STILL CONNECTED - NO CAR"));
        }
    }

    uint32_t ms = millis();
    int32_t yHueDelta32 = ((int32_t)cos16( ms * (27/1) ) * (350 / kMatrixWidth));
    int32_t xHueDelta32 = ((int32_t)cos16( ms * (39/1) ) * (310 / kMatrixHeight));
    DrawOneFrame( ms / 65536, yHueDelta32 / 32768, xHueDelta32 / 32768);
    if( ms < 5000 ) {
      FastLED.setBrightness( scale8( BRIGHTNESS, (ms * 256) / 5000));
    } else {
      FastLED.setBrightness(BRIGHTNESS);
    }
    FastLED.show();

}

