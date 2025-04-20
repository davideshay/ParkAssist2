#ifndef PARKASSIST_H
#define PARKASSIST_H

#include <FastLED.h>

#define TEMP_SENSOR_BUS 14
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
  TIMER_EXPIRED
};

#define VL53L8CX_TARGET_STATUS_RANGE_VALID 5
#define VL53L8CX_TARGET_STATUS_RANGE_VALID_LARGE_PULSE 9
#define VL53L8CX_TARGET_STATUS_RANGE_VALID_NO_PREVIOUS 10

double getSensorDistancemm();

#endif
  