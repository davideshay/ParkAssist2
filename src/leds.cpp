#include <Arduino.h>
#include <FastLED.h>
#include <parkassist.h>
#include <WebSerial.h>

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

// constant coordinates to show specific features {X, Y} == 

static const int negSignCoord[] = { 2,0};
static const int greaterSignCoord[] = { 1,0};
static const int firstDigitCoord[] = {3,0};
static const int secondDigitCoord[] = {7,0};
static const int carLogoCoord[] = {1,1};
static const int whiteBarTopCoord[] = {11,2};
static const int whiteBarBottomCoord[] = {11,13};
static const int redBoxCoord[] = {12,0};
static const int greenBoxCoord[] = {12,3};
static const int yellowBoxCoord[] = {12,6};

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
  int16_t digit = digits3x5[digit];
  for (size_t x = 0; x < 3; x++)
  {
    for (size_t y = 0; y < 5; y++)
    {
      bool pixel = getBit(digit,(x*3)+y);
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

void drawPictureGuide(carInfoStruct currentCar, double currentDistance, boolean carDetected) {
  drawWhiteBar(whiteBarTopCoord[0],whiteBarTopCoord[1]);
  drawWhiteBar(whiteBarBottomCoord[0],whiteBarBottomCoord[1]);
  enum colorCodes {
    RED,YELLOW,GREEN
  };
  colorCodes colorCode;
  if (carDetected) {colorCode = GREEN;};


  
};
