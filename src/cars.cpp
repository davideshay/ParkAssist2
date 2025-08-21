#include <cars.h>
#include <parkassist.h>

// structureof carLogo - 1 blank bit, followed by 3 bits (color 0-7) each 5x to represent first 5 pixels. 
// Cover 60 pixels -- 6 rows x 10 columns. Go row-by-row in order. So first int_16 is first 5 pixels.
// Second int_16 is 5 remaining pixels in first row, then on to 2nd row.

const carInfoStruct cars[] = {
// R1S
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
    CRGB::Purple },
    .carBeaconAddress = NimBLEAddress("7e:c5:d6:4a:d4:bb",0)
},
// Tesla 3
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
      CRGB::Purple },
    .carBeaconAddress = NimBLEAddress("b0:f8:f2:d9:39:85",0)
},

// Tesla Y

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
      CRGB::Purple },
    .carBeaconAddress = NimBLEAddress("84:ee:03:f8:ad:02",0)
},
};

const carInfoStruct defaultCar = cars[0];
carInfoStruct currentCar = defaultCar;
