#include <Arduino.h>
#include <parkassist.h>

void initLEDs();
void DrawOneFrame( uint8_t startHue8, int8_t yHueDelta8, int8_t xHueDelta8);
uint16_t XYsafe( uint8_t x, uint8_t y);
uint16_t XY( uint8_t x, uint8_t y);
void drawLEDs();
void drawDistance(double currentDistance);
void drawDistanceWord(boolean useMetric);
void drawCarLogo(char carLetter);
void drawPictureGuide(carInfoStruct currentCar, double currentDistance, boolean carDetected);
