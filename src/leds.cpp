#include <Arduino.h>
#include <FastLED.h>
#include <parkassist.h>
#include <WebSerial.h>
#include <leds.h>

#define COLOR_ORDER RGB
#define CHIPSET     WS2812
#define BRIGHTNESS 64

#define LED_PANEL_PIN 47

// Params for width and height
const uint8_t kMatrixWidth = 16;
const uint8_t kMatrixHeight = 16;

// Param for different pixel layouts
const bool    kMatrixSerpentineLayout = true;
const bool    kMatrixVertical = false;
#define NUM_LEDS (kMatrixWidth * kMatrixHeight)
CRGB leds_plus_safety_pixel[ NUM_LEDS + 1];
CRGB* const leds( leds_plus_safety_pixel + 1);

void initLEDs() {
    FastLED.addLeds<CHIPSET, LED_PANEL_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalSMD5050);
    FastLED.setBrightness( BRIGHTNESS );
}

uint16_t XY( uint8_t x, uint8_t y)
{
  uint16_t i;
  
  if( kMatrixSerpentineLayout == false) {
    if (kMatrixVertical == false) {
      i = (y * kMatrixWidth) + x;
    } else {
      i = kMatrixHeight * (kMatrixWidth - (x+1))+y;
    }
  }

  if( kMatrixSerpentineLayout == true) {
    if (kMatrixVertical == false) {
      if( y & 0x01) {
        // Odd rows run backwards
        uint8_t reverseX = (kMatrixWidth - 1) - x;
        i = (y * kMatrixWidth) + reverseX;
      } else {
        // Even rows run forwards
        i = (y * kMatrixWidth) + x;
      }
    } else { // vertical positioning
      if ( x & 0x01) {
        i = kMatrixHeight * (kMatrixWidth - (x+1))+y;
      } else {
        i = kMatrixHeight * (kMatrixWidth - x) - (y+1);
      }
    }
  }
  return i;
}

uint16_t XYsafe( uint8_t x, uint8_t y)
{
  if( x >= kMatrixWidth) return -1;
  if( y >= kMatrixHeight) return -1;
  return XY(x,y);
}

bool getBit(uint16_t input, int position) // position in range 0-15 (critically going from left-bit (15) to right-bit (0))
{
    if (position > 15 || position < 0) {
      WebSerial.println(F("Error getting bit, negative or > 15"));
      return 0;}
    int actualPos = 15 - position; 
    return (input >> actualPos) & 0x1;
}

void drawADigit(int digit, int startX, int startY, CRGB color) {
  if (digit < 0 || digit > 9) {return;}
  int16_t digitMap = digits3x5[digit];
  for (size_t x = 0; x < 3; x++)
  {
    for (size_t y = 0; y < 5; y++)
    {
      bool pixel = getBit(digitMap,(x*3)+y);
      leds[ XYsafe(startX + x, startY + y)] = color;
    }
  }
}

void drawFirstDigit(int digit, CRGB color) {
  drawADigit(digit,firstDigitCoord[0],firstDigitCoord[1],color);
};

void drawSecondDigit(int digit, CRGB color) {
  drawADigit(digit,secondDigitCoord[0],secondDigitCoord[1],color);  
};

void drawInfiniteDistance() {
    drawFirstDigit(9,CRGB::White);
    drawSecondDigit(9,CRGB::White);
};

void drawNegativeSign(CRGB color) {
  leds[ XYsafe(negSignCoord[0],negSignCoord[1])] = color;
  leds[ XYsafe(negSignCoord[0]+1,negSignCoord[1])] = color;
}

void drawDistance(double currentDistance,distanceEvaluation distEval) {
  if (abs(currentDistance) > 99) {
    drawInfiniteDistance();
    return;
  }
  int absDistance = currentDistance;
  if (currentDistance < 0) {
    drawNegativeSign(distEval.colorRGB);
  }
  absDistance=abs(currentDistance);
  if (absDistance < 10) {
    drawFirstDigit(absDistance,distEval.colorRGB);
    return;
  }
  drawFirstDigit(absDistance / 10,distEval.colorRGB);  // Get the tens place
  drawSecondDigit(absDistance % 10,distEval.colorRGB);  // Get the units place
};

void drawDistanceWord(boolean useMetric,distanceEvaluation distEval) {
  for (size_t y = 0; y < 3; y++)
  {
    for (size_t x = 0; x < 9; x++)
    {
      uint16_t row;
      if (useMetric) {row = word_cm_9x3[y];} else {row = word_in_9x3[y];}
      bool pixel = getBit(row,7+x);
      //TODO
    }
  }
};

void drawCarLogo(carInfoStruct currentCar) {
  // loop through columns and rows, getting the right pixel from the logo array
  for (size_t x = 0; x < 10; x++)
  {
    for (size_t y = 0; y < 6; y++)
    {
      int16_t pixelColSet;
      if (x < 5) { pixelColSet = currentCar.carLogo[y*2];} else 
                 { pixelColSet = currentCar.carLogo[(y*2)+1];};
      int pixelColor=getBit(pixelColSet,x % 5);
      CRGB pixelRGBColor = currentCar.logoColors[pixelColor];
      leds[ XYsafe(carLogoCoord[0]+x,carLogoCoord[1]+y)] = pixelRGBColor;
    }
  }
};

void drawWhiteBar(int x, int y) {
  for (size_t i = 0; i < 5; i++)
  {
    leds[ XYsafe(x+i,y)] = CRGB::White;
  }
}

void drawColorBox(int initX, int initY, CRGB color) {
  for (size_t x = 0; x < 3; x++)
  {
    for (size_t y = 0; y < 9; y++)
    {
      leds[ XYsafe(x+initX,y+initY)] = color;
    }
  }
};

void drawPictureGuide(distanceEvaluation distEval) {
  drawWhiteBar(whiteBarTopCoord[0],whiteBarTopCoord[1]);
  drawWhiteBar(whiteBarBottomCoord[0],whiteBarBottomCoord[1]);
  switch (distEval.colorCode)
  {
  case RED:
    drawColorBox(redBoxTopLowestCoord[0],redBoxTopLowestCoord[1]-distEval.colorOffset,distEval.colorRGB);
    break;
  case GREEN:
    drawColorBox(greenBoxTopLowestCoord[0],greenBoxTopLowestCoord[1]-distEval.colorOffset,distEval.colorRGB);
    break;
  case YELLOW:
    drawColorBox(yellowBoxTopLowestCoord[0],yellowBoxTopLowestCoord[1]-distEval.colorOffset,distEval.colorRGB);
    break;
  default:
    break;
  };
};
