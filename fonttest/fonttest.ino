#include <SPI.h>


#include <Ethernet.h>
#include <EthernetClient.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <vincent.h>


#define PIN 6

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(64, PIN, NEO_GRB + NEO_KHZ800);

byte mac[]    = {  0x41, 0x7c, 0x8f, 0x5a, 0xa4, 0x70 };
byte server[] = { 85,119,83,194 };
byte message[] = "Happy Christmas";
byte buffer[400];
int length = sizeof(message);
void callback(char* topic, byte* payload, unsigned int length) {
  updateBuffer(payload, length<50?length:50);
}

//EthernetClient ethClient;
//PubSubClient client(server, 1883, callback, ethClient);

void setup() {
  updateBuffer(message, length);
//  Serial.begin(19200);
//  Serial.print(Ethernet.begin(mac)>0?"got dhcp":"failed dhcp");
//
//  if (client.connect("arduinoClient")) {
////      Serial.println("connected");
//    } else {
////      Serial.println("failed to connect");
//    }
//    
//    client.publish("/pzf/status", "MQTT connected ok");
//    client.subscribe("/pzf/message");
    
    
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  // Some example procedures showing how to display to the pixels:
//  char string[] = {'h','e','L','l','o',' ','D','A','N','!'};

  spell(100);
}


uint8_t getPixel(uint8_t x, uint8_t y) {
  return (x*8)+y;
}

void character(uint8_t ch, uint8_t col) {
   for (int y = 0; y<8; ++y) {
     
     for (int x = 0; x<8; ++x) {
        char b = vincent_data[ch][y];
        if (b & (1<<x)) {
//           Serial.print(1);  
           strip.setPixelColor(getPixel(x,y),Wheel(((x * 8) + y) & 255));
        }
        else 
        {
//           Serial.print(0);
           strip.setPixelColor(getPixel(x,y),0);
        }
      }
      strip.show();
      delay(10);
//      Serial.println();
   }
//   strip.show(); 
}

void updateBuffer(byte string[], uint8_t len) {
  byte *p = &buffer[0];
  for (int i=0; i<len & i<50; ++i) {
    memcpy(p, vincent_data[string[i]], 8);
    p +=8;
  }
}

void spell(uint8_t wait) {

    for (int pointer = 0; pointer < length; ++pointer)
    {
      for (int i=0; i<8; ++i) {
         int row = pointer*8+i;
         splatrow(row,i);
      }
      strip.show();
      delay(wait);
    }
  
}

void splatrow(uint8_t row, uint8_t x) {
  for (int y = 0; y<8; ++y) {
      if (buffer[row] & (1<<y)) {  
           strip.setPixelColor(getPixel(x,y),Wheel(((x * 8) + y) & 255));
        }
        else 
        {
           strip.setPixelColor(getPixel(x,y),0);
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

