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
    
    
    
    
    
    
This is the circuit for a simple unisolated DMX Shield for input:

 Arduino                    SN 75176 A or MAX 481CPA
 pin       3k        1k      _______________
 |   GND---/\/\/\-+-/\/\/\--| 1      Vcc 8 |------ +5v
 V                |         |              |                 DMX Output
  RX |------------+    +----| 2        B 7 |---------------- Pin 2
     |                 |    |              |
     |      +3.3v------_----| 3 DE     A 6 |---------------- Pin 3
     |                      |              |
TX(2)|----------------------| 4 DI   Gnd 5 |---+------------ Pin 1
     |                       _______________   |
     |          330 ohm                       GND
  14 |-----------/\/\/\-----[ LED ]------------|


!) Pins 2 & 3 of the MAX481 are held HIGH to enable output
!) The 1k/3k resistors are a simple voltage divider to convert from 5v to 3.3v (for input only)

!!) Note that ESP8266 has limited UARTs and this library will conflict with Serial.
!!) Also, depending on the board, the above circuit may need to be disconnected
    in order to load a sketch.

*/
/**************************************************************************/
#include <LXESP8266UARTDMX.h>

#define LED_PIN_OUTPUT 14
#define DIRECTION_PIN 5

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
 ESP8266DMX.setSlot(8,level);
 ESP8266DMX.setSlot(1,level);
 analogWrite(LED_PIN_OUTPUT, level);
 delay(50);
 level+= d;
 if ( level == 0 ) {
  d = 1;
 } else if ( level == 100 ) {
  d = -1;
 }
 
}
