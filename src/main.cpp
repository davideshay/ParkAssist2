#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <WiFiClient.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include <ElegantOTA.h>
#include <wificodes.h>
#include <HCSR04.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <esp_camera.h>
#include <leds.h>
#include <camera.h>
#include <parkassist.h>
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
// #define LED_PANEL_PIN 47

// structureof carLogo - 1 blank bit, followed by 3 bits (color 0-7) each 5x to represent first 5 pixels. 
// Cover 60 pixels -- 6 rows x 10 columns. Go row-by-row in order. So first int_16 is first 5 pixels.
// Second int_16 is 5 remaining pixels in first row, then on to 2nd row.

carInfoStruct defaultCar = 
  { .targetFrontDistanceCm = 60, .maxFrontDistanceCm = 30, .lengthOffsetCm = 0, .carLogo = 
    {
      0b0001001001000000,
      0b0001000001001001,
      0b0001000001000001,
      0b0001000001000000,
      0b0001001001000000,
      0b0001000001000000,
      0b0001001000000000,
      0b0001000001001001,
      0b0001000001000000,
      0b0001000000000001,
      0b0001000001000000,
      0b0001000001001001
    },
    .logoColors = {
      CRGB::Black,
      CRGB::White,
      CRGB::Red,
      CRGB::Green,
      CRGB::Blue,
      CRGB::Yellow,
      CRGB::Orange,
      CRGB::Purple }
  };

carInfoStruct currentCar;

enum stateOpts {
  BASELINE,
  DOOR_OPENING,
  CAR_PRESENT,
  DETECTING_CAR_TYPE,
  CAR_TYPE_DETECTED,
  SHOWING_DATA,
  TIMER_EXPIRED
};

stateOpts curState = BASELINE;

int secsToReset = 90;
unsigned long camera_checking_millis = 0;
int maxCameraCheckMillis = 5000;
unsigned long timer_started_millis = 0;

AsyncWebServer serverOTA(80);
AsyncWebServer serverLog(81);
AsyncWebServer serverCam(82);

UltraSonicDistanceSensor distanceSensor(DIST_SENSOR_TRIGGER, DIST_SENSOR_ECHO);  // Initialize sensor that uses digital pins 42 and 41

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(TEMP_SENSOR_BUS);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature tempSensors(&oneWire);

float currentTemp;
double currentDistance;
distanceEvaluation currentDistanceEvaluation;
double carFirstDetectedDistanceFromFront;
boolean carFirstDetected = false;
double carFirstClearedSensorDistanceFromFront;
boolean carFirstClearedSensor = false;
boolean carDetected;
boolean useMetric = false;

unsigned long ota_progress_millis = 0;

void onOTAStart() {
  Serial.println("OTA update started!");
}

void onOTAProgress(size_t current, size_t final) {
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
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
  Serial.println("OTA HTTP server started");

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
  initLEDs();
  
  serverCam.on("/picture", HTTP_GET, [](AsyncWebServerRequest * request) {
    camera_fb_t * frame = NULL;
    frame = esp_camera_fb_get();
    request->send_P(200, "image/jpeg", (const uint8_t *)frame->buf, frame->len);
    esp_camera_fb_return(frame);
  });
  serverCam.begin();

}

distanceEvaluation evaluateDistance() {
  // TODO -- does not deal with lengthOffsets for other cars
  distanceEvaluation distEval = {
    .colorRGB = CRGB::Red,
    .colorCode = RED,
    .colorOffset = 0
  };
  if (digitalRead(IR_BREAK_SENSOR) == LOW) {
    carDetected = true;
    if (!carFirstDetected) {
      carFirstDetected = true;
      carFirstDetectedDistanceFromFront = currentDistance;
      WebSerial.print(F("First Detected Car at distance: "));
      WebSerial.println(carFirstDetectedDistanceFromFront);
    }
  } else {
    if (carDetected) {
      carFirstClearedSensor = true;
      carFirstClearedSensorDistanceFromFront = currentDistance;
      WebSerial.print(F("First Detected Car Cleared Sensor at distance: "));
      WebSerial.println(carFirstClearedSensorDistanceFromFront);
    }
    carDetected = false;
  }
  if (carDetected) {
    distEval.colorRGB = CRGB::Yellow;
    distEval.colorCode = YELLOW;
    // car is at 120cm, first detected car at 200cm, target = 90 --> 110 cm range from first detected to target
    // how far inside = 200-120 = 80cm, so 80/110 = 45% of range, offset should be 2
    distEval.colorOffset = constrain(((carFirstDetectedDistanceFromFront - currentDistance) / (carFirstDetectedDistanceFromFront - currentCar.targetFrontDistanceCm))*yellowBoxMaxOffset,0,yellowBoxMaxOffset);
  };
  if (currentDistance < currentCar.targetFrontDistanceCm) {
    distEval.colorRGB = CRGB::Red;
    distEval.colorCode = RED;
    // car is at 80cm, target=90, max=60 --> 30 cm range from target to max
    // how far inside that range = 90-80 = 10cm, so using 30% of range, offset should be 1
    distEval.colorOffset = constrain((((currentCar.targetFrontDistanceCm - currentDistance) / (currentCar.targetFrontDistanceCm - currentCar.maxFrontDistanceCm))*redBoxMaxOffset),0,redBoxMaxOffset);
  } else {
    // car cleared sensor at 100cm, car is now at 95cm, target is 90 --> 10cm range from cleared->target
    // how far inside that range = 100-95 = 5cm/10cm --> offset = 2
    distEval.colorOffset = constrain(((carFirstClearedSensorDistanceFromFront - currentDistance) / (carFirstClearedSensorDistanceFromFront - currentCar.targetFrontDistanceCm)*greenBoxMaxOffset),0,greenBoxMaxOffset);
    distEval.colorRGB = CRGB::Green;
    distEval.colorCode = GREEN;
  }
  return distEval;
}

void getCurrentData() {
  tempSensors.requestTemperatures(); 
  currentTemp = tempSensors.getTempCByIndex(0);
  currentDistance = distanceSensor.measureDistanceCm(currentTemp);  
  currentDistanceEvaluation = evaluateDistance();
};

void displayCurrentData() {
  FastLED.clear();
  CRGB color;
  drawDistance(currentDistance,currentDistanceEvaluation);
  drawDistanceWord(useMetric,currentDistanceEvaluation);
  drawCarLogo(currentCar);
  drawPictureGuide(currentDistanceEvaluation);
  FastLED.show();
}

void loop() {
    static unsigned long last_print_time = millis();
    ElegantOTA.loop();
    WebSerial.loop();

    switch (curState) {
      case BASELINE:
        if (digitalRead(IR_BREAK_SENSOR) == LOW) {
          WebSerial.println(F("IR Sensor Broken from default. Car Present"));
          timer_started_millis = millis();
          carDetected = true;
          curState = CAR_PRESENT;
        }
        break;
      case CAR_PRESENT:
        curState = DETECTING_CAR_TYPE;
        camera_checking_millis = millis();
        WebSerial.println(F("Now Detecting Car Type..."));
        break;
      case DETECTING_CAR_TYPE:
        if (millis() - camera_checking_millis > maxCameraCheckMillis) {
          WebSerial.println(F("Car Detection Timeout, set to default"));
          currentCar = defaultCar;
          curState = CAR_TYPE_DETECTED;
        }
        break;
      case CAR_TYPE_DETECTED:
        curState = SHOWING_DATA;
        WebSerial.println(F("Car Detected. Showing Data..."));
        timer_started_millis = millis();
        break;
      case SHOWING_DATA:
        getCurrentData();
        displayCurrentData();
        if (((millis() - timer_started_millis) > (unsigned long)(secsToReset * 1000))  && !carDetected) {
            WebSerial.println(F("Timer expired and no car Detected"));
            curState = TIMER_EXPIRED;
        }
        break;
      case TIMER_EXPIRED:
        WebSerial.println(F("Timer expired, turning off display and resetting to baseline"));
        FastLED.clear();
        FastLED.show();
        curState = BASELINE;
        carFirstDetected = false;
        carFirstDetectedDistanceFromFront = 0;
        carFirstClearedSensor = false;
        carFirstClearedSensorDistanceFromFront = 0;
    }

    if ((unsigned long)(millis() - last_print_time) > 3000) {
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
}

