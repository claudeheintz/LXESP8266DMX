/* LXESP8266UARTDMX.h
   Copyright 2015 by Claude Heintz Design

Copyright (c) 2015, Claude Heintz
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of LXESP8266DMX nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

