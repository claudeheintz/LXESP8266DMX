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

#define LED_PIN_OUTPUT 14
#define DIRECTION_PIN 15

uint8_t level = 0;
int d = 1;

void setup() {
  pinMode(LED_PIN_OUTPUT, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);

  Serial.setDebugOutput(UART0); //use uart0 for debugging

  ESP8266DMX.setDirectionPin(DIRECTION_PIN);
  ESP8266DMX.startOutput();
}

/************************************************************************

  The main loop fades the levels of addresses 7 and 8 to full
  
*************************************************************************/

void loop() {
 ESP8266DMX.setSlot(7,level);
 ESP8266DMX.setSlot(10,level);
 analogWrite(LED_PIN_OUTPUT, level);
 delay(50);
 level+= d;
 if ( level == 0 ) {
  d = 1;
 } else if ( level == 100 ) {
  d = -1;
 }
 
}
