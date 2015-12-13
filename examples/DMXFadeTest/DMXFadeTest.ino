/**************************************************************************/
/*!
    @file     DMXFadeTest.ino
    @author   Claude Heintz
    @license  BSD (see LXESP8266DMX LICENSE)
    @copyright 2015 by Claude Heintz

    Simple Fade test of RESP8266 DMX Driver
    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/
#include <LXESP8266UARTDMX.h>

// LX8266DMXOutput instance
LX8266DMXOutput* dmx_output = new LX8266DMXOutput();

uint8_t level = 0;

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);
  Serial.begin(9600);
  Serial.setDebugOutput(1); //use uart0 for debugging
   
  delay(1000);        //avoid boot print??
  dmx_output->start();
}

/************************************************************************

  The main loop fades the levels of addresses 7 and 8 to full
  
*************************************************************************/

void loop() {
 dmx_output->setSlot(7,level);
 dmx_output->setSlot(8,level);
 delay(50);
 level++;
}
