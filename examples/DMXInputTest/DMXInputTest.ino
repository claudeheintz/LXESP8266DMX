/**************************************************************************/
/*!
    @file     DMXInputTest.ino
    @author   Claude Heintz
    @license  BSD (see LXESP8266DMX LICENSE)
    @copyright 2015 by Claude Heintz

    Control brightness of LED on GPIO14 with DMX address 1
    @section  HISTORY

    v1.00 - First release
    v1.01 - Updated for single LX8266DMX class
    
    
    
    
    
    
This is the circuit for a simple unisolated DMX Shield for input:

 Arduino                    SN 75176 A or MAX 481CPA
 pin       3k        1k      _______________
 |   GND---/\/\/\-+-/\/\/\--| 1      Vcc 8 |------ +5v
 V                |         |              |                 DMX Input
  RX |------------+    +----| 2        B 7 |---------------- Pin 2
     |                 |    |              |
     |      GND-------------| 3 DE     A 6 |---------------- Pin 3
     |                      |              |
TX(2)|----------------------| 4 DI   Gnd 5 |---+------------ Pin 1
     |                       _______________   |
     |          330 ohm                       GND
  14 |-----------/\/\/\-----[ LED ]------------|


!) Pins 2 & 3 of the MAX481 are held LOW to enable input
!) The 1k/3k resistors are a simple voltage divider to convert from 5v to 3.3v

*/
/**************************************************************************/
#include <LXESP8266UARTDMX.h>

int got_dmx = 0;

void setup() {
  WiFi.forceSleepBegin(); //not using WiFi, sleep to prevent background activity
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
  //wdt_reset();
}
