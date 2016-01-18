/**************************************************************************/
/*!
    @file     DMXInputTest.ino
    @author   Claude Heintz
    @license  BSD (see LXESP8266DMX LICENSE)
    @copyright 2015 by Claude Heintz

    SControl brightness of LED on GPIO14 with DMX address 1
    @section  HISTORY

    v1.00 - First release
    v1.01 - Updated for single LX8266DMX class
*/
/**************************************************************************/
#include <LXESP8266UARTDMX.h>

int got_dmx = 0;

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(14, OUTPUT);
  ESP8266DMX.setDataReceivedCallback(&gotDMXCallback);
  delay(1000);        //avoid boot print??
  ESP8266DMX.startInput();
}


// ***************** input callback function *************

void gotDMXCallback(int slots) {
  got_dmx = slots;
}

/************************************************************************

  The main loop checks to see if dmx input is available (got_dmx>0)
  And then reads the level of dimmer 1 to set PWM level of LED connected to pin 14
  
*************************************************************************/

void loop() {
  if ( got_dmx ) {
    //ESP8266 PWM is 10bit 0-1024
    analogWrite(14,2*ESP8266DMX.getSlot(1));
  }
}
