#ifndef PARKASSIST_H
#define PARKASSIST_H

#include <FastLED.h>

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

#endif
  