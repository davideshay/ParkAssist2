#ifndef LEDS_H
#define LEDS_H

#include <Arduino.h>
#include <parkassist.h>
#include <FastLED.h>

void initLEDs();
uint16_t XYsafe( uint8_t x, uint8_t y);
uint16_t XY( uint8_t x, uint8_t y);
void drawDistance(double currentDistance, boolean useMetric, distanceEvaluation distEval, carInfoStruct currentCar);
void drawDistanceWord(boolean useMetric, distanceEvaluation distEval);
void drawCarLogo(carInfoStruct car);
void drawPictureGuide(distanceEvaluation distEval);

// constant coordinates to show specific features {X, Y} == 

static const int negSignCoord[] = { 0,2};
static const int greaterSignCoord[] = { 0,1};
static const int firstDigitCoord[] = {3,0};
static const int secondDigitCoord[] = {7,0};
static const int distanceWordCoord[] = {1,6};
static const int carLogoCoord[] = {0,10};
static const int whiteBarTopCoord[] = {11,2};
static const int whiteBarBottomCoord[] = {11,13};
static const int redBoxTopLowestCoord[] = {12,2};
static const int redBoxMaxOffset = 2; // (zero-based, so could be 0, 1 or 2)
static const int greenBoxTopLowestCoord[] = {12,4};
static const int greenBoxMaxOffset = 1; // (zero-based so could be 0 or 1)
static const int yellowBoxTopLowestCoord[] = {12,6};
static const int yellowBoxMaxOffset = 2; // (zero-based, so could be 0, 1 or 2)

// structure - 1 blank bit followed by 3 bits for each 5 rows = 16 bits
static const uint16_t digits3x5[] = {
  0b0111101101101111,
  0b0110010010010111,
  0b0111001111100111,
  0b0111001111001111,
  0b0101101111001001,
  0b0111100111001111,
  0b0111100111101111,
  0b0111001001001001,
  0b0111101111101111,
  0b0111101111001111
};

// structure - 7 blank bit followed by 9 bits for the content of the first row = 16 bits
static const uint16_t word_cm_9x3[] = {
  0b0000000111011110,
  0b0000000100010101,
  0b0000000111010101
};

// structure - 7 blank bit followed by 9 bits for the content of the first row = 16 bits
static const uint16_t word_in_9x3[] = {
  0b0000000000010110,
  0b0000000000010101,
  0b0000000000010101
};


#endif