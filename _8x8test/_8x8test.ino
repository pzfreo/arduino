#include <Adafruit_NeoPixel.h>

#define PIN 6

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(64, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  // Some example procedures showing how to display to the pixels:
  bounce(70);
  colorWipe(strip.Color(255, 0, 0), 10); // Red
  colorWipe(strip.Color(0, 255, 0), 10); // Green
  colorWipe(strip.Color(0, 0, 255), 10); // Blue
  rainbow(5);
  rainbowCycle(5);
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

uint8_t getPixel(uint8_t x, uint8_t y) {
  return (x*8)+y;
}

void bounce(uint8_t wait) {
  uint8_t col = 0;
  uint8_t xdir = 1;
  uint8_t ydir = -1;
  int8_t x = 1, y = 5;
  
  while (true) {
  boolean jump = false;
  int8_t ox, oy;
  int8_t oox, ooy;
  int8_t ooox, oooy;
  ooox = oox; oooy=ooy;
  oox = ox; ooy=oy;
  ox = x; oy=y;
  if (random(100)>85) {xdir = random(3)-1;}
  x = x+xdir;
  if (x>7) { x = 7; xdir = -1;}
  if (x<0) { x = 0; xdir = 1;}
  if (random(100)>85) { ydir = random(3)-1;}
  y+=ydir;
  if (y>7) {y=7; ydir = -1;}
  if (y<0) {y=0; ydir = 1;}
  col = (col+1);
//  if (jump) x+=xdir;
  strip.setPixelColor(getPixel(x,y),Wheel(col));

  
  if (!(ooox==x && oooy==y))  strip.setPixelColor(getPixel(ooox,oooy),0);
  strip.show();
  delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

