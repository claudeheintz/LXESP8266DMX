/**************************************************************************/
/*!
    @file     DMXFadeTest.ino
    @author   Claude Heintz
    @license  BSD (see LXESP8266DMX LICENSE)
    @copyright 2017 by Claude Heintz

    Simple Fade test of ESP8266 DMX Driver and second UART output
    @section  HISTORY

    v1.00 - First release

*/
/**************************************************************************/
#include <LXESP8266UARTDMX.h>
#include "LXESP8266UART0DMX.h"
#include <uart.h>

uint8_t level = 0;

void setup() {
  uart_set_debug(UART_NO); //disable debug print
  pinMode(14, OUTPUT);
  delay(1000);        //avoid boot print??
  
  ESP8266DMX0.startDualOutput();
}

/************************************************************************

  The main loop fades the levels of addresses 7 and 8 to full
  
*************************************************************************/

void loop() {
 ESP8266DMX.setSlot(7,level);
 ESP8266DMX.setSlot(8,255);
 ESP8266DMX0.setSlot(5,level);
 ESP8266DMX0.setSlot(8,255);
 delay(50);
 level++;
}
