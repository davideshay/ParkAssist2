#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <WiFiClient.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include <wificodes.h>
#include <HCSR04.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <esp_camera.h>
#include <leds.h>
#include <camera.h>
#include <parkassist.h>
#include <log.h>
#include <FastLED.h>
#include <PrettyOTA.h>
#include "FS.h"
#include <LittleFS.h>
#include <SimpleKalmanFilter.h>

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
  { .targetFrontDistanceCm = 77, .maxFrontDistanceCm = 50, .lengthOffsetCm = 0, .sensorDistanceFromFrontCm = 550,
     .carLogo = 
    {
      0b0100100100000000,
      0b0100000100100100,
      0b0100000100000100,
      0b0100000100000000,
      0b0100100100000000,
      0b0100000100000000,
      0b0100100000000000,
      0b0100000100100100,
      0b0100000100000000,
      0b0100000000000100,
      0b0100000100000000,
      0b0100000100100100
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

int secsToReset = 60;
unsigned long camera_checking_millis = 0;
int maxCameraCheckMillis = 1000;
unsigned long timer_started_millis = 0;
unsigned long logging_millis = 0;
unsigned long dist_check_millis = 0;
const unsigned long time_between_dist_checks_millis = 60;
unsigned long temp_check_millis = 0;
const unsigned long time_between_temp_checks_millis = 60000;
unsigned long ota_update_millis = 0;
const unsigned long time_between_ota_log_millis = 500;
SimpleKalmanFilter kalmanFilter(75,75,3);

AsyncWebServer serverOTA(80);
PrettyOTA OTAUpdates;
AsyncWebServer serverLog(81);
AsyncWebServer serverCam(82);
AsyncWebServer serverLogDetail(83);
WiFiClient logClient;
IPAddress logTarget = IPAddress(10,10,1,136);

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

boolean fileLogging = false;
boolean netLogging = true;
boolean webLogging = true;
#define FORMAT_LITTLEFS_IF_FAILED true
File logFile;
boolean okToLog = true;

boolean demoMode = false;
double demoDistance;

void onOTAStart(NSPrettyOTA::UPDATE_MODE updateMode)
{
    WebSerial.println("OTA update started");
    if(updateMode == NSPrettyOTA::UPDATE_MODE::FIRMWARE)
        WebSerial.println("Mode: Firmware");
    else if(updateMode == NSPrettyOTA::UPDATE_MODE::FILESYSTEM)
        WebSerial.println("Mode: Filesystem");
}

void onOTAProgress(uint32_t currentSize, uint32_t totalSize)
{
    if (millis() - ota_update_millis < time_between_dist_checks_millis) {return;}
    WebSerial.printf("OTA Progress Current: %u bytes, Total: %u bytes\n", currentSize, totalSize);
}

void onOTAEnd(bool success) {
  if (success) {
    WebSerial.println("OTA update finished successfully!");
  } else {
    WebSerial.println("There was an error during OTA update!");
  }
}

void openLogFileAppend() {
  if (fileLogging) {
    logFile = LittleFS.open("/logfile.txt", FILE_APPEND);
    if (!logFile) {
      WebSerial.println("Unable to open log file for writing");
      okToLog = false;
    }  
  }
}

void openLogFileRead() {  
  logFile = LittleFS.open("/logfile.txt", FILE_READ);
  if (!logFile) {
    WebSerial.println("Unable to open log file for reading");
    okToLog = false;
  }
}

void logData(String message, bool includeWeb = true) {
  if (webLogging && includeWeb) {WebSerial.println(message);}
  if (fileLogging && okToLog) {logFile.println(message);}
  if (netLogging) {logClient.println(message);}
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
 
  serverLog.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is WebSerial demo. You can access the WebSerial interface at http://" + WiFi.localIP().toString() + "/webserial");
  });
 
  serverOTA.begin();
  Serial.println("OTA HTTP server started");

  serverOTA.begin();
  OTAUpdates.Begin(&serverOTA);
  OTAUpdates.OverwriteAppVersion("1.0.0");
  PRETTY_OTA_SET_CURRENT_BUILD_TIME_AND_DATE();
  OTAUpdates.OnStart(onOTAStart);
  OTAUpdates.OnProgress(onOTAProgress);
  OTAUpdates.OnEnd(onOTAEnd);

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
    if (d == "move") {
      WebSerial.println("Detected move command, setting to car detected");
      curState = CAR_TYPE_DETECTED;
    } else if (d.startsWith("changeip") && netLogging) {
      String newIPs = d.substring(9);
      WebSerial.println("Changing netlogging IP address to " + newIPs );
      IPAddress newIP;
      newIP.fromString(newIPs);
      logClient.stop();
      if (!logClient.connect(newIP,10000)) {
        WebSerial.println("Error changing netlogging IP address to " + newIPs);
      } else {
        logClient.println("Changed netlogging to this IP address: "+newIPs);
      }
    }
    WebSerial.println(d);
  });
  delay(1000);
  WebSerial.println("Starting Web Serial Log for Park Assist");

  if (netLogging) {
    if (!logClient.connect(logTarget, 10000)) {      
      WebSerial.println("Connection to host failed");
    }
    logClient.println("TCP Logging Initiated");
    WebSerial.println("TCP Logging Initiated");
  }


  serverLog.begin();
  tempSensors.begin();
  initCamera();
  initLEDs();
  
  serverCam.on("/picture", HTTP_GET, [](AsyncWebServerRequest * request) {
    camera_fb_t * frame = NULL;
    frame = esp_camera_fb_get();
    request->send(200, "image/jpeg", (const uint8_t *)frame->buf, frame->len);
    esp_camera_fb_return(frame);
  });
  serverCam.begin();

  if (fileLogging) {
    if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
      WebSerial.println("Unable to initialize LittleFS");
      okToLog = false;
    } else {
      LittleFS.remove("/logfile.txt");
      logFile = LittleFS.open("/logfile.txt", FILE_APPEND);
      openLogFileAppend();
    }  
  }

  // Handle the download button
  serverLogDetail.on("/download", HTTP_GET, [](AsyncWebServerRequest *request){
    logFile.close();
    openLogFileRead();
    request->send(LittleFS, "/logfile.txt", String(),false);
    logFile.close();
    openLogFileAppend();
  });
  serverLogDetail.begin();

  //TODO -- DELETE LATER
  currentCar = defaultCar;
}

void logCurrentData() {
  if (millis() - logging_millis < 500) {return;}
  boolean carPresent = false;
  if (digitalRead(IR_BREAK_SENSOR) == LOW) {
    carPresent = true;
  }
  String logLine;
  logLine = "cctcm=";
  logLine += currentCar.targetFrontDistanceCm;
  logLine += " dcm=";
  logLine += currentDistance;
  logLine += " dttcm=";
  logLine += currentDistanceEvaluation.cmToTarget;
  logLine += " dttin=";
  logLine += currentDistanceEvaluation.inchesToTarget;
  logLine += " dspdt=";
  logLine += currentDistanceEvaluation.displayDistance;
  logLine += " dspinf=";
  logLine += currentDistanceEvaluation.displayInfinity;
  if (carPresent) {
    logLine += " CARPRES";
  } else {
    logLine += " NOCAR";
  }
  logLine += " cc=";
  logLine += currentDistanceEvaluation.colorCode;
  logLine += " co=";
  logLine += currentDistanceEvaluation.colorOffset;
  logData(logLine,true);
  logging_millis=millis();
}

distanceEvaluation evaluateDistance() {
  // TODO -- does not deal with lengthOffsets for other cars
  distanceEvaluation distEval = {
    .colorRGB = CRGB::Blue,
    .colorCode = BLUE,
    .colorOffset = 0,
    .displayInfinity = false,
    .inchesToTarget = 0,
    .cmToTarget = 0,
    .displayDistance = 0
  };
  double smoothedDistance = currentDistance;
  if (currentDistance == -1) {
    smoothedDistance = currentCar.sensorDistanceFromFrontCm;
  };
  if (digitalRead(IR_BREAK_SENSOR) == LOW) {
    carDetected = true;
    if (!carFirstDetected) {
      carFirstDetected = true;
      carFirstDetectedDistanceFromFront = smoothedDistance;
      String msg = "First Detected Car at distance: ";
      msg += carFirstDetectedDistanceFromFront;
      logData(msg);
    }
  } else {
    if (carDetected) {
      carFirstClearedSensor = true;
      carFirstClearedSensorDistanceFromFront = smoothedDistance;
      String msg = "First Detected Car Cleared Sensor at distance: ";
      msg += carFirstClearedSensorDistanceFromFront;
      logData(msg);
    }
    carDetected = false;
  }
  if (carDetected) {
    distEval.colorRGB = CRGB::Yellow;
    distEval.colorCode = YELLOW;
    // car is at 120cm, first detected car at 200cm, target = 90 --> 110 cm range from first detected to target
    // how far inside = 200-120 = 80cm, so 80/110 = 45% of range, offset should be 2
    distEval.colorOffset = constrain(((carFirstDetectedDistanceFromFront - smoothedDistance) / (carFirstDetectedDistanceFromFront - currentCar.targetFrontDistanceCm))*yellowBoxMaxOffset,0,yellowBoxMaxOffset);
  } else {
    if (smoothedDistance < currentCar.targetFrontDistanceCm) {
      distEval.colorRGB = CRGB::Red;
      distEval.colorCode = RED;
      // car is at 80cm, target=90, max=60 --> 30 cm range from target to max
      // how far inside that range = 90-80 = 10cm, so using 30% of range, offset should be 1
      distEval.colorOffset = constrain((((currentCar.targetFrontDistanceCm - smoothedDistance) / (currentCar.targetFrontDistanceCm - currentCar.maxFrontDistanceCm))*redBoxMaxOffset),0,redBoxMaxOffset);
    } else {
      // car cleared sensor at 100cm, car is now at 95cm, target is 90 --> 10cm range from cleared->target
      // how far inside that range = 100-95 = 5cm/10cm --> offset = 2
      distEval.colorOffset = constrain(((carFirstClearedSensorDistanceFromFront - smoothedDistance) / (carFirstClearedSensorDistanceFromFront - currentCar.targetFrontDistanceCm)*greenBoxMaxOffset),0,greenBoxMaxOffset);
      distEval.colorRGB = CRGB::Green;
      distEval.colorCode = GREEN;
    }  
  }
  distEval.cmToTarget = smoothedDistance - currentCar.targetFrontDistanceCm;
  distEval.inchesToTarget = (distEval.cmToTarget / 2.54);
  if (useMetric) {
    if (distEval.cmToTarget > 99 || distEval.cmToTarget < -99) {
      distEval.displayInfinity = true;}
    else {
      distEval.displayDistance = distEval.cmToTarget;
    }
  }
  else {
    if (distEval.inchesToTarget > 99 || distEval.inchesToTarget < -99) {
      distEval.displayInfinity = true;}
    else {
      distEval.displayDistance = distEval.inchesToTarget;
    }
  }
  return distEval;
}

void logDetailData(double od,double cd,float ed) {
  if (!fileLogging && !webLogging && !netLogging) {return;}
  String logLine;
  logLine = "ms=";
  logLine += millis();
  logLine += " od=";
  logLine += od;
  logLine += " cd=";
  logLine += cd;
  logLine += " ed=";
  logLine += ed;
  logData(logLine,false);
}

void getTemperature() {
  tempSensors.requestTemperatures(); 
  currentTemp = tempSensors.getTempCByIndex(0);
}

void getCurrentData() {
  if (millis() - dist_check_millis < time_between_dist_checks_millis) {return;}
  double origDistance = distanceSensor.measureDistanceCm(currentTemp);
  double corrDistance = origDistance;
  if (corrDistance == -1) {corrDistance = currentCar.sensorDistanceFromFrontCm;}
  float estimatedDistance = kalmanFilter.updateEstimate(corrDistance);
  logDetailData(origDistance,corrDistance,estimatedDistance);
  currentDistance = estimatedDistance;
  currentDistanceEvaluation = evaluateDistance();
  dist_check_millis = millis();
};

void displayCurrentData() {
  logCurrentData();
  FastLED.clear();
  drawDistance(currentDistance,useMetric,currentDistanceEvaluation,currentCar);
  drawDistanceWord(useMetric,currentDistanceEvaluation);
  drawCarLogo(currentCar);
  drawPictureGuide(currentDistanceEvaluation);
  FastLED.show();
}

void resetBaseline() {
  FastLED.clear();
  FastLED.show();
  curState = BASELINE;
  carFirstDetected = false;
  carFirstDetectedDistanceFromFront = 0;
  carFirstClearedSensor = false;
  carFirstClearedSensorDistanceFromFront = 0;
  logFile.close();
}

void loop() {
    WebSerial.loop();
    if (millis() - temp_check_millis > time_between_temp_checks_millis) {
      getTemperature();
      temp_check_millis = millis();
    }

    switch (curState) {
      case BASELINE:
        if (digitalRead(IR_BREAK_SENSOR) == LOW) {
          logData("IR Sensor Broken from default. Car Present");
          timer_started_millis = millis();
          carDetected = true;
          curState = CAR_PRESENT;
          openLogFileAppend();
        }
        break;
      case CAR_PRESENT:
        curState = DETECTING_CAR_TYPE;
        camera_checking_millis = millis();
        logData("Now Detecting Car Type...");
        break;
      case DETECTING_CAR_TYPE:
        if (millis() - camera_checking_millis > maxCameraCheckMillis) {
          logData("Car Detection Timeout, set to default");
          currentCar = defaultCar;
          curState = CAR_TYPE_DETECTED;
        }
        break;
      case CAR_TYPE_DETECTED:
        curState = SHOWING_DATA;
        logData("Car Detected. Showing Data...");
        timer_started_millis = millis();
        logging_millis = millis();
        break;
      case SHOWING_DATA:
        getCurrentData();
        displayCurrentData();
        if (((millis() - timer_started_millis) > (unsigned long)(secsToReset * 1000))  && !carDetected) {
            logData("Timer expired and no car Detected");
            curState = TIMER_EXPIRED;
        }
        break;
      case TIMER_EXPIRED:
        logData("Timer expired, turning off display and resetting to baseline");
        resetBaseline();        
    }

}

