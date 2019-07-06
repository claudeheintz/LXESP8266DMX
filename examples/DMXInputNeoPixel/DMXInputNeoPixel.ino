/**************************************************************************/
/*!
    @file     DMXInputTest.ino
    @author   Claude Heintz
    @license  BSD (see LXESP8266DMX LICENSE)
    @copyright 2019 by Claude Heintz

    Control Neo-Pixels with DMX input
    Demonstrates single frame read using ESP8266DMX.isReceiving()
    
    @section  HISTORY

    v1.00 - First release 
*/
/**************************************************************************/
#include <LXESP8266UARTDMX.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define DIRECTION_PIN 4

// data pin for NeoPixels
#define PIN 14
// NUM_OF_NEOPIXELS, max of 170/RGB or 128/RGBW
#define NUM_OF_NEOPIXELS 12
// LEDS_PER_NEOPIXEL, RGB = 3, RGBW = 4
#define LEDS_PER_NEOPIXEL 3
// see Adafruit NeoPixel Library for options to pass to Adafruit_NeoPixel constructor
Adafruit_NeoPixel ring = Adafruit_NeoPixel(NUM_OF_NEOPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// min sometimes raises compile error. see https://github.com/esp8266/Arduino/issues/263 solution seems to be using _min
const int total_pixels = _min(512, LEDS_PER_NEOPIXEL * NUM_OF_NEOPIXELS);
byte pixels[NUM_OF_NEOPIXELS][LEDS_PER_NEOPIXEL];

// flag for dmx data received
int got_dmx = 0;

void setup() {
  pinMode(DIRECTION_PIN, OUTPUT);		// pin allows control of tranceiver chip direction
  digitalWrite(DIRECTION_PIN, LOW);		// low is input
  
  ESP8266DMX.setDataReceivedCallback(&gotDMXCallback);
  delay(1000);        //avoid boot print??

  ring.begin();
  setPixel(0,255);   // initial power up indicator 1 red pixel
  sendPixels();
}


// ***************** input callback function *************

void gotDMXCallback(int slots) {
  got_dmx = slots;
}

// ***************** DMX read single frame *************

#define DMX_READ_TIMEOUT_COUNT 10000

void readDMXFrame() {   // <- Attempt to read a single DMX frame
  ESP8266DMX.startInput();
  
  uint32_t t_o_ct = 0;  // timeout counter should wait no more than 2 frames of DMX
                        // (? good value for limit)
                        // --will delay animation if too long
                        // --will prevent read if too short
  while ( (got_dmx == 0 ) && (t_o_ct < DMX_READ_TIMEOUT_COUNT) ) {
    t_o_ct++;
    if ( ESP8266DMX.isReceiving() ) {       // if started to read packet, wait until done
      t_o_ct = 0;                           // still timeout in case of DMX signal interruption
      while ( ESP8266DMX.isReceiving() && (got_dmx == 0 ) && (t_o_ct < DMX_READ_TIMEOUT_COUNT) ) {
        t_o_ct++;
      }

      t_o_ct = 0;
    }   //<- receiving
  }     //<- ! got_dmx || timeout

  ESP8266DMX.stop();
}


// ***************** sends pixel buffer to ring *************

void sendPixels() {
  uint16_t r,g,b;
  for (int p=0; p<NUM_OF_NEOPIXELS; p++) {
    r = pixels[p][0];
    g = pixels[p][1];
    b = pixels[p][2];
    r = (r*r)/255;    //gamma correct
    g = (g*g)/255;
    b = (b*b)/255;
    if ( LEDS_PER_NEOPIXEL == 3 ) {
    	ring.setPixelColor(p, r, g, b);		//RGB
    } else if ( LEDS_PER_NEOPIXEL == 4 ) {
    	uint16_t w = pixels[p][3];
    	w = (w*w)/255;
    	ring.setPixelColor(p, r, g, b, w);	//RGBW
    }
  }
  ring.show();
}

// ***************** set the dmx value in the pixel buffer *************

void setPixel(uint16_t index, uint8_t value) {	
  uint8_t pixel = index/LEDS_PER_NEOPIXEL;
  uint8_t color = index%LEDS_PER_NEOPIXEL;
  pixels[pixel][color] = value;
}

/************************************************************************

  The main loop tries to read a single DMX frame
  It then checks to see if slots were read (got_dmx>0)
  If so, it sends the values to the NEoPixels
  
*************************************************************************/

void loop() {
  readDMXFrame();
  
  if ( got_dmx ) {
    for (int i=0; i<total_pixels; i++) {
    	setPixel(i, ESP8266DMX.getSlot(i+1));
    }
    sendPixels();
    got_dmx = 0;
  }

}
