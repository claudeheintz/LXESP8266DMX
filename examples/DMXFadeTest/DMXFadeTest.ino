/**************************************************************************/
/*!
    @file     DMXFadeTest.ino
    @author   Claude Heintz
    @license  BSD (see LXESP8266DMX LICENSE)
    @copyright 2015 by Claude Heintz

    Simple Fade test of RESP8266 DMX Driver
    @section  HISTORY

    v1.00 - First release
    v1.01 - Updated for single LX8266DMX class
*/
/**************************************************************************/
#include <LXESP8266UARTDMX.h>

uint8_t level = 0;

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);
  Serial.begin(9600);
  Serial.setDebugOutput(1); //use uart0 for debugging
   
  delay(1000);        //avoid boot print??
  ESP8266DMX.startOutput();
}

/************************************************************************

  The main loop fades the levels of addresses 7 and 8 to full
  
*************************************************************************/

void loop() {
 ESP8266DMX.setSlot(7,level);
 ESP8266DMX.setSlot(8,level);
 delay(50);
 level++;
}
