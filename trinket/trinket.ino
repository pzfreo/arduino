//#include <SPI.h>
//
//
//#include <Ethernet.h>
//#include <EthernetClient.h>
//#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <vincent.h>


#define PIN 2

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(64, PIN, NEO_GRB + NEO_KHZ800);

byte message[] = "*** Merry Christmas from the Fremantle Family ***";
byte buffer[500];
int length = sizeof(message);

void setup() {
  updateBuffer(message, length);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  spell(100);
}


uint8_t getPixel(uint8_t x, uint8_t y) {
  return (x*8)+y;
}

void updateBuffer(byte string[], uint8_t len) {
  byte *p = &buffer[0];
  for (int i=0; i<=len & i<51; ++i) {
     memcpy(p, vincent_data[string[i%len]], 8);
     p +=8;
  }
}

void spell(uint8_t wait) {

    for (int pointer = 0; pointer < 8*length; ++pointer)
    {
      for (int i=0; i<8; ++i) {
         int row = pointer+i;
         splatrow(row,i);
      }
      strip.show();
      delay(wait);
    }
  
}

void splatrow(uint8_t col, uint8_t x) {
  for (int y = 0; y<8; ++y) {
      if (buffer[col] & (1<<y)) {  
           strip.setPixelColor(getPixel(y,x),Wheel(((x * 8) + y) & 255));
        }
        else 
        {
           strip.setPixelColor(getPixel(y,x),0);
        }
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

