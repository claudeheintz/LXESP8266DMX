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

uint8_t level = 0;
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

  The main loop fades the levels of addresses 7 and 8 to full
  
*************************************************************************/

void loop() {
  if ( got_dmx ) {
    //on ESP8266 PWM is 10bit 0-1024
    analogWrite(14,2*dmx_input->getSlot(1));
  }
}
