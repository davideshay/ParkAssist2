#ifndef PARKASSIST_H
#define PARKASSIST_H

#include <FastLED.h>

struct carInfoStruct {
    int targetFrontDistanceCm;
    int maxFrontDistanceCm;
    int lengthOffsetCm;
    uint16_t carLogo[12];
    CRGB logoColors[8];
  };

enum colorCodes {
    RED,YELLOW,GREEN
  };

// return values from evaluate distance function
// RGB and enum of color, and colorOffset is a value from 0-2 representing summarized version of distance

struct distanceEvaluation {
    CRGB colorRGB;
    colorCodes colorCode;
    int colorOffset;
};

#endif
  