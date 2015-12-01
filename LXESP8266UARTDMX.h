/* LXESP8266UARTDMX.h
   Copyright 2015 by Claude Heintz Design

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
-----------------------------------------------------------------------------------

   The LXESP8266UARTDMX library supports output and input of DMX using the UART
   serial output of an ESP8266 microcontroller.  LXESP8266UARTDMX uses
   UART1 instead of UART0 for output.  This means that hardware Serial
   can still be used for communication.  (do not use Serial1)
   
   This is the circuit for a simple unisolated DMX Shield
   that could be used with LXESP8266UARTDMX.  It uses a line driver IC
   to convert the output from the ESP8266 to DMX:

 ESP8266 Pin
 |                         SN 75176 A or MAX 481CPA
 V                            _______________
       |                      | 1      Vcc 8 |------(+5v)
       |       |              |              |              DMX Output
       |                 +----| 2        B 7 |---------------- Pin 2
       |                 |    |              |
 (4/5) |------(or +5v)---+----| 3 DE     A 6 |---------------- Pin 3
       |                      |              |
TX (2) |----------------------| 4 DI   Gnd 5 |---+------------ Pin 1
       |                                         |
       |                                       (GND)

*/


#ifndef LX8266DMX_H
#define LX8266DMX_H

#include <Arduino.h>
#include <inttypes.h>

#define DMX_MIN_SLOTS 24
#define DMX_MAX_SLOTS 512

typedef void (*LXRecvCallback)(int);

class LX8266DMXOutput {

  public:
	LX8266DMXOutput ( void );
	/*!
    * @param pin used to control driver chip's DE (data enable) line, HIGH for output
    * @param slots number of slots aka channels or addresses (~24-512)
   */
	LX8266DMXOutput ( uint8_t pin, uint16_t slots  );
    ~LX8266DMXOutput ( void );
    
   /*!
    * @brief starts interrupt that continuously sends DMX output
    * @discussion Sets up baud rate, bits and parity, 
    *             sets globals accessed in ISR, 
    *             enables transmission and tx interrupt.
   */
   void start( void );
   /*!
    * disables transmission and tx interrupt
   */
	void stop( void );
	
	/*!
	 * @brief Sets the number of slots (aka addresses or channels) sent per DMX frame.
	 * @discussion defaults to 512 or DMX_MAX_SLOTS and should be no less DMX_MIN_SLOTS slots.  
	 *             The DMX standard specifies min break to break time no less than 1024 usecs.  
	 *             At 44 usecs per slot ~= 24
	 * @param slot the highest slot number (~24 to 512)
	*/
	void setMaxSlots (int slot);
	/*!
	 * @brief Sets the output value of a slot
	 * @param slot number of the slot aka address or channel (1-512)
	 * @param value level (0-255)
	*/
   void setSlot (int slot, uint8_t value);
   /*!
    * @brief provides direct access to data array
    * @return pointer to dmx array
   */
   uint8_t* dmxData(void);
   
   /*!
    * @brief UART tx empty interrupt handler
   */
  	void		_tx_empty_irq(void);
	void		_tx_empty_irq1(void);
    
  private:

   /*!
   * @brief represents phase of sending dmx packet data/break/etc used to change baud settings
   */
  	uint8_t  _dmx_state;
   /*!
    * @brief true when ISR is enabled
   */
  	uint8_t  _interrupt_status;
   /*!
   * @brief count of idle interrupts
   */
  	uint8_t  _idle_count;
   /*!
   * @brief number of dmx slots ~24 to 512
   */
  	uint16_t  _current_slot;
   /*!
   * @brief number of dmx slots ~24 to 512
   */
  	uint16_t  _slots;
   /*!
   * @brief Array of dmx data including start code
   */
  	uint8_t  _dmxData[DMX_MAX_SLOTS+1];
  	
};

