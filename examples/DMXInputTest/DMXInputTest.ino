/**************************************************************************/
/*!
    @file     DMXInputTest.ino
    @author   Claude Heintz
    @license  BSD (see LXESP8266DMX LICENSE)
    @copyright 2015 by Claude Heintz

    SControl brightness of LED on GPIO14 with DMX address 1
    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/
#include <LXESP8266UARTDMX.h>

// LX8266DMXInput instance
LX8266DMXInput* dmx_input = new LX8266DMXInput();

int got_dmx = 0;

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(14, OUTPUT);
  dmx_input->setDataReceivedCallback(&gotDMXCallback);
  delay(1000);        //avoid boot print??
  dmx_input->start();
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
    analogWrite(14,2*dmx_input->getSlot(1));
  }
}
