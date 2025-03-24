#include <Arduino.h>
#include <parkassist.h>

void initLEDs();
uint16_t XYsafe( uint8_t x, uint8_t y);
uint16_t XY( uint8_t x, uint8_t y);
void drawDistance(double currentDistance, distanceEvaluation distEval);
void drawDistanceWord(boolean useMetric, distanceEvaluation distEval);
void drawCarLogo(carInfoStruct car);
void drawPictureGuide(carInfoStruct currentCar, double currentDistance, boolean carDetected);
