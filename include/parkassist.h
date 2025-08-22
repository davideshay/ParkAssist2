#ifndef PARKASSIST_H
#define PARKASSIST_H

#include <FastLED.h>
#include <Preferences.h>
#include <log.h>
#include <lidar.h>
#include <nvs_flash.h>
#include <camera.h>
#include <wifiota.h>
#include <NimBLEAddress.h>

#define SDA_PIN 19
#define SCL_PIN 20
#define IR_BREAK_SENSOR 21
#define LED_PANEL_PIN 47

int64_t esp_millis();

struct carInfoStruct {
    int targetFrontDistanceCm;
    int maxFrontDistanceCm;
    int lengthOffsetCm;
    int sensorDistanceFromFrontCm;
    uint16_t carLogo[12];
    CRGB logoColors[8];
    NimBLEAddress carBeaconAddress;
  };

enum CarType {
  R1S=0,
  TESLAM3=1,
  TESLAMY=2,
  NUM_CARTYPES=3
};

enum colorCodes {
    RED,YELLOW,GREEN,WHITE,BLUE
  };

// return values from evaluate distance function
// RGB and enum of color, and colorOffset is a value from 0-2 representing summarized version of distance

struct distanceEvaluation {
    CRGB colorRGB;
    colorCodes colorCode;
    int colorOffset;
    bool displayInfinity;
    int16_t inchesToTarget;
    int16_t cmToTarget;
    int8_t displayDistance;
};

enum stateOpts {
  BASELINE,
  DOOR_OPENING,
  CAR_PRESENT,
  DETECTING_CAR_TYPE,
  CAR_TYPE_DETECTED,
  SHOWING_DATA,
  TIMER_EXPIRED,
  TIMER_EXPIRED_CAR_PRESENT,
  TIMER_EXPIRED_WAIT_TO_CLEAR,
};

struct ParkPreferences {
  uint8_t struct_version; // version of this struct, so we can update it in the future
  int64_t maxCameraCheckMillis;
  int64_t timeBetweenWifiChecksMillis;
  uint16_t logPort;
  uint16_t secsToResetCarStillPresent;
  uint16_t secsToResetAfterCleared;
  bool fileLogging;
  bool netLogging;
  bool webLogging;
  bool serialLogging;
};

struct CalibrationPreferences {
  bool calibrationDataSaved;
  VL53L4CX_CalibrationData_t calData;
};

void getPreferences();
void logPrefs(ParkPreferences logPrefs, CalibrationPreferences logCalPrefs);
void setPreferences();
void setOnePref(String msg);
void clearPreferences();
void clearNVSAndReboot();
void resetBaseline();

/// --- temp
void copyPrefsIntoCalData();
void updatePrefsVersion();

#endif
  