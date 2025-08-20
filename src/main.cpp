#include <Arduino.h>
#include <parkassist.h>
#include <SimpleKalmanFilter.h>
#include <filter.h>
#include <leds.h>
#include <ble.h>

// 
// PIN LAYOUT -- all defined in parkassist.h
// 
// Camera uses pins 4,5,6,7,15,16,17,18,8,9,10,11,12,13
//    (all of one side of the board except for 3 JTAG and 46 LOG
//
// #define SDA_PIN 19 I2C bus used for VL53L4CX distance sensor - SDA and SCL
// #define SCL_PIN 20 
// #define IR_BREAK_SENSOR 21
// #define LED_PANEL_PIN 47. (WS2812 panel)

// structureof carLogo - 1 blank bit, followed by 3 bits (color 0-7) each 5x to represent first 5 pixels. 
// Cover 60 pixels -- 6 rows x 10 columns. Go row-by-row in order. So first int_16 is first 5 pixels.
// Second int_16 is 5 remaining pixels in first row, then on to 2nd row.

carInfoStruct defaultCar = 
  { .targetFrontDistanceCm = 83, .maxFrontDistanceCm = 60, .lengthOffsetCm = 0, .sensorDistanceFromFrontCm = 550,
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

ParkPreferences parkPreferences;
extern ParkPreferences defaultPreferences;
CalibrationPreferences calibrationPreferences;

stateOpts curState = BASELINE;

int64_t camera_checking_millis = 0;
int64_t reset_present_millis = 0;
int64_t reset_after_cleared_millis = 0;
int64_t dist_check_millis = 0;
const int64_t time_between_dist_checks_millis = 120;
SimpleKalmanFilter kalmanFilter(75,75,3);
ExponentialFilter<float> distanceFilter(65,defaultCar.sensorDistanceFromFrontCm);
uint64_t wifi_check_millis = 0;
uint64_t ble_check_millis = 0;

extern bool otaStarted;

double currentDistance;
const double maxDistanceDeltaOK = 50;
distanceEvaluation currentDistanceEvaluation;
double carFirstDetectedDistanceFromFront;
bool carFirstDetected = false;
double carFirstClearedSensorDistanceFromFront;
bool carFirstClearedSensor = false;
double realDistanceDetected = false;
bool carDetected;
bool useMetric = false;
bool carClearedSensorTimerStarted = false;

bool demoMode = false;
double demoDistance;
bool demoIRBREAK = false;

bool testModeNoIR = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(IR_BREAK_SENSOR, INPUT);
  Serial.begin(115200);

  initWifi();
  initOTA();
  
  if (!otaStarted) {
    getPreferences();
    initLogging();
  }
  
  logData("OTA Updates Enabled, starting LIDAR sensor",true,true);

  if (!otaStarted) {initLidarSensor();};
  if (!otaStarted) {initCamera();};
  if (!otaStarted) {initLEDs();};
  if (!otaStarted) {initBLE();};
  
  logData("Camera and LEDs initialized, starting baseline loop", true,true);
  
  //TODO -- DELETE LATER
  currentCar = defaultCar;
}

void evaluateDistance() {
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
  if (currentDistance == -1 || currentDistance < 0) {
    smoothedDistance = currentCar.sensorDistanceFromFrontCm;
  };
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
  currentDistanceEvaluation = distEval;
}

void getIRBreak() {
  bool irBreak;
  if (demoMode) {irBreak = demoIRBREAK;} else {irBreak = digitalRead(IR_BREAK_SENSOR) == LOW;}
  if (irBreak) {
    carDetected = true;
    if (!carFirstDetected) {
      carFirstDetected = true;
      carFirstDetectedDistanceFromFront = currentDistance;
      String msg = "First Detected Car at distance: ";
      msg += carFirstDetectedDistanceFromFront;
      logData(msg,true);
    }
  } else {
    if (carDetected) {
      carFirstClearedSensor = true;
      carFirstClearedSensorDistanceFromFront = currentDistance;
      String msg = "First Detected Car Cleared Sensor at distance: ";
      msg += carFirstClearedSensorDistanceFromFront;
      logData(msg,true);
    }
    carDetected = false;
  }
}

void getCurrentData() {
//  if (esp_millis() - dist_check_millis < time_between_dist_checks_millis) {return;}
//  dist_check_millis = esp_millis();
  getIRBreak();
  if (demoMode) {
    currentDistance = demoDistance;
  } else {
    DistanceResults distanceResults = getSensorDistance();

    double corrDistance = -1;
    bool defaultDistance = false;
    if (distanceResults.distanceStatus != DISTANCE_OK) {
      if (realDistanceDetected) {
        // At some point, I had already detected a real distance, but now I don't
        // This is either bad sensor data (99% of the time) or could be car backing out and can no longer be seen
        evaluateDistance();
        return;
      } else {
      // Never had a real distance detected , default value to the max distance in the garage (distance to door)
        corrDistance = currentCar.sensorDistanceFromFrontCm;
        defaultDistance = true;
      }
    } else {
      // A "real" distance has come in from the sensor. If it deviates by more than 50cm from the current distance
      // then throw it out if this is not the first reading
      if (realDistanceDetected) {
        // if (abs(currentDistance - origDistance) > 50) {return;}
      } else {
        // This is the first real distance to be detected -- accept the value without deviation check
        realDistanceDetected = true;
      }
      corrDistance = distanceResults.distance_mm / 10;
    }
    if (defaultDistance) {
      logData("Default distance used, no real distance detected",false);
      currentDistance = corrDistance;
    } else {
      float estimatedDistance = kalmanFilter.updateEstimate(corrDistance);
      distanceFilter.Filter(corrDistance);
      logDetailData(distanceResults.distance_mm,corrDistance,estimatedDistance,distanceFilter.Current());
//      currentDistance = estimatedDistance;
      currentDistance = distanceFilter.Current();
    }
  }
  evaluateDistance();
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
  blankDisplay();
  curState = BASELINE;
  carFirstDetected = false;
  carFirstDetectedDistanceFromFront = 0;
  carFirstClearedSensor = false;
  carFirstClearedSensorDistanceFromFront = 0;
  realDistanceDetected = false;
  currentDistance = 0;
  distanceFilter.SetCurrent(defaultCar.sensorDistanceFromFrontCm);
  closeLogFile();
  carDetected = false;
  resetLidarBaseline();
}

void checkReconnectWiFi() {
  if (esp_millis() - wifi_check_millis > parkPreferences.timeBetweenWifiChecksMillis) {
    if ((WiFi.status() != WL_CONNECTED)) {
      logData("Reconnecting to WiFi...",true);
      disconnectNetLogging();
      WiFi.disconnect();
      WiFi.reconnect();
      connectNetLogging();
      logData("WiFi reconnected. Logging re-initiated.",true);
    }
    wifi_check_millis = esp_millis();
  }
}  

void checkBleLoop() {
  if (esp_millis() - ble_check_millis > 2000) {
    bleLoop();
    ble_check_millis = esp_millis();
  }
}

void resetPresentClearedTimers() {
  reset_present_millis = esp_millis();
  reset_after_cleared_millis = esp_millis();
  carClearedSensorTimerStarted = false;
}

void loop() {
    if (otaStarted) {
      delay(2000);
      return;
    }
    checkReconnectWiFi();
    checkBleLoop();
    WebSerial.loop();
    switch (curState) {
      case BASELINE:
        if (digitalRead(IR_BREAK_SENSOR) == LOW && !testModeNoIR) {
          logData("IR Sensor Broken from default. Car Present",true);
          carDetected = true;
          curState = CAR_PRESENT;
          openLogFileAppend();
        }
        break;
      case CAR_PRESENT:
        curState = DETECTING_CAR_TYPE;
        camera_checking_millis = esp_millis();
        logData("Now Detecting Car Type...",true);
        break;
      case DETECTING_CAR_TYPE:
        if (esp_millis() - camera_checking_millis > parkPreferences.maxCameraCheckMillis) {
          logData("Car Detection Timeout, set to default",true);
          currentCar = defaultCar;
          curState = CAR_TYPE_DETECTED;
        }
        break;
      case CAR_TYPE_DETECTED:
        curState = SHOWING_DATA;
        logData("Car Detected. Showing Data...",true);
        startSensorRanging();
        resetPresentClearedTimers();
        break;
      case SHOWING_DATA:
        getCurrentData();
        displayCurrentData();
        if ((esp_millis() - reset_present_millis) > (parkPreferences.secsToResetCarStillPresent * 1000)) {
          if (carDetected) {
            logData("Car still detected, turning off display and waiting to clear.",true);
            curState = TIMER_EXPIRED_CAR_PRESENT;          
          }
          break;
        }
        if (carDetected && carClearedSensorTimerStarted) {
          resetPresentClearedTimers();
          break;
        }
        if (!carDetected && !carClearedSensorTimerStarted) {
          reset_after_cleared_millis = esp_millis();
          carClearedSensorTimerStarted = true;
          logData("Car cleared sensor, starting after cleared timer",true);
          break;
        }
        if (carClearedSensorTimerStarted && ((esp_millis() - reset_after_cleared_millis) > (parkPreferences.secsToResetAfterCleared * 1000))) {
          logData("Car cleared sensor, resetting to baseline",true);
          curState = TIMER_EXPIRED;
        }
        break;
      case TIMER_EXPIRED:
        logData("Timer expired, turning off display and resetting to baseline",true);
        resetBaseline();
      case TIMER_EXPIRED_CAR_PRESENT:
        logData("Timer expired car still present, turn off display now.");
        resetLidarBaseline();
        blankDisplay();
        curState = TIMER_EXPIRED_WAIT_TO_CLEAR;
      case TIMER_EXPIRED_WAIT_TO_CLEAR:
        getIRBreak();
        if (!carDetected) {
          logData("Timer had expired with car still present. Now car not present, resetting to baseline",true);
          resetBaseline();
        }
    }

}

