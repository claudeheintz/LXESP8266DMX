/**************************************************************************/
/*!
    @file     LXESP8266UART0DMX.cpp
    @author   Claude Heintz
    @license  BSD (see LXESP8266UARTDMX.h)
    @copyright 2017 by Claude Heintz

    DMX Driver for ESP8266 using UART0.

    @section  HISTORY

    v1.0 - Alternative second universe output using UART0
           in addition to LXESP8266UARTDMX which uses 
           TX - UART1
           RX - UART0
           
           Warning:  The ESP8266 will write debugging messages to UART0
           			 prior to your code executing.  This may produce garbled DMX
           			 on startup if the output driver is enabled prior to
           			 the startup() method of your sketch.
*/
/**************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"
#include "cbuf.h"

extern "C" {
#include "osapi.h"
#include "ets_sys.h"
#include "mem.h"
#include "user_interface.h"
}

#include "LXESP8266UART0DMX.h"
#include "LXESP8266UARTDMX.h"

LX8266U0DMX ESP8266DMX0;


/* ***************** Utility functions derived from ESP HardwareSerial.cpp  ****************
   HardwareSerial.cpp - esp8266 UART support - Copyright (c) 2014 Ivan Grokhotkov. All rights reserved.
   HardwareSerial is licensed under GNU Lesser General Public License
   HardwareSerial is included in the esp8266 by ESP8266 Community board package for Arduino IDE
*/

//these constants are now defined in the Arduino ESP8266 library v2.1.0
//static const int UART0 = 0;
//static const int UART1 = 1;
//static const int UART_NO = -1;


/**
 *  UART GPIOs
 *
 *  UART0 is used by Serial for debug output
 *        so UART1 is used for DMX output (tx)
 *  UART1 does not have the rx pin available on most ESP modules so
 *        UART0 is used for DMX input (rx)
 *
 *
 * UART0 TX: 1 or 2				SPECIAL or FUNCTION_4
 * UART0 RX: 3						SPECIAL
 *
 * UART0 SWAP TX: 15
 * UART0 SWAP RX: 13
 *
 *
 * UART1 TX: 2 or 7 (NC)		SPECIAL or FUNCTION_4
 * UART1 RX: 8 (NC)				FUNCTION_4
 *
 * UART1 SWAP TX: 11 (NC)
 * UART1 SWAP RX: 6 (NC)
 *
 * NC = Not Connected to Module Pads --> No Access
 *      Pins 6-11 typically connected to flash on most modules
 *      see http://arduino.esp8266.com/versions/1.6.5-1160-gef26c5f/doc/reference.html
 */
 
void uart0_tx_interrupt_handler(LX8266U0DMX* dmxo);
void uart_dual_tx_interrupt_handler(LX8266U0DMX* dmxo);
void uart0_tx_flush(void);
void uart0_enable_tx_interrupt(LX8266U0DMX* dmxo);
void uart_dual_enable_tx_interrupt(LX8266U0DMX* dmxo);
void uart0_enable_dual_tx_interrupt(LX8266U0DMX* dmxo);
void uar0t_disable_tx_interrupt(void);
void uart_dual_disable_tx_interrupt(void);
void uart0_set_baudrate(int uart_nr, int baud_rate);

void uart0_init_tx(int baudrate, byte config, LX8266U0DMX* dmxo);

void uart0_uninit_tx(void);
void uart_dual_uninit_tx(void);

// ####################################################################################################

// UART register definitions see esp8266_peri.h

ICACHE_RAM_ATTR void uart0_tx_interrupt_handler(LX8266U0DMX* dmxo) {

    // -------------- UART 0 --------------
    // check uart status register 
    // if fifo is empty clear interrupt
    // then call _tx_empty_irq
	  if(U0IS & (1 << UIFE)) {
			U0IC = (1 << UIFE);
			dmxo->txEmptyInterruptHandler();
	  }
	 
}

ICACHE_RAM_ATTR void uart_dual_tx_interrupt_handler(LX8266U0DMX* dmxo) {
    digitalWrite(14,LOW);
    
    if(U0IS & (1 << UIFE)) {
			U0IC = (1 << UIFE);
			dmxo->txEmptyInterruptHandler();
	  }
    
    
	  if(U1IS & (1 << UIFE)) {
			U1IC = (1 << UIFE);
			ESP8266DMX.txEmptyInterruptHandler();
	  }

   digitalWrite(14,HIGH);
}


// ####################################################################################################

//LX uses uart0 for tx
void uart0_tx_flush(void) {
    uint32_t tmp = 0x00000000;

    tmp |= (1 << UCTXRST);
    
    USC0(UART0) |= (tmp);
    USC0(UART0) &= ~(tmp);
}

// ------------- uart0_enable/disable TX functions
//LX uses uart0 for tx
void uart0_enable_tx_interrupt(LX8266U0DMX* dmxo) {
	USIC(UART0) = 0x1ff;								//clear interrupts	
	ETS_UART_INTR_ATTACH(&uart0_tx_interrupt_handler, dmxo);
   USIE(UART0) |= (1 << UIFE);						// enable fifo empty interrupt
   ETS_UART_INTR_ENABLE();							// enables all UART interrupts!
}

void uart_dual_enable_tx_interrupt(LX8266U0DMX* dmxo) {
	USIC(UART0) = 0x1ff;								//clear interrupts
	ETS_UART_INTR_ATTACH(&uart_dual_tx_interrupt_handler, dmxo);
   USIE(UART0) |= (1 << UIFE);						// enable fifo empty interrupt
   ETS_UART_INTR_ENABLE();							// enables all UART interrupts!
}

//LX uses uart0 for tx
void uart0_disable_tx_interrupt(void) {
   USIE(UART0) &= ~(1 << UIFE);
   //ETS_UART_INTR_DISABLE();		disables all UART interrupts including Hardware serial
}

void uart_dual_disable_tx_interrupt(void) {
   USIE(UART0) &= ~(1 << UIFE);
   USIE(UART1) &= ~(1 << UIFE);
}

// ------------- uart_set functions

//LX uses uart0 for tx
void uart0_set_baudrate(int uart_nr, int baud_rate) {
    USD(uart_nr) = (ESP8266_CLOCK / baud_rate);
}

//LX uses uart0 for tx
void uart0_set_config(int uart_nr, byte config) {
    USC0(uart_nr) = config;
}

// ------------- uart_init function

void uart_dual_init_tx(int baudrate, byte config, LX8266U0DMX* dmxo) {
  ESP8266DMX.startOutput();
	ETS_UART_INTR_DISABLE();	//start ESP8266DMX then disable interrupt
	
	pinMode(1, SPECIAL);
	uint32_t conf1 = 0x00000000;
	
    uart0_set_baudrate(UART0, baudrate);
    USC0(UART0) = config;
    uart0_tx_flush();
    uart_dual_enable_tx_interrupt(dmxo);

    //conf1 |= (0x00 << UCFET);// tx empty threshold is zero
    						   // tx fifo empty interrupt triggers continuously unless
    						   // data register contains a byte which has not moved to shift reg yet
    USC1(UART0) = conf1;
}

void uart0_init_tx(int baudrate, byte config, LX8266U0DMX* dmxo) {
	pinMode(1, SPECIAL);
	uint32_t conf1 = 0x00000000;
	
    uart0_set_baudrate(UART0, baudrate);
    USC0(UART0) = config;
    uart0_tx_flush();
    uart0_enable_tx_interrupt(dmxo);

    //conf1 |= (0x00 << UCFET);// tx empty threshold is zero
    						   // tx fifo empty interrupt triggers continuously unless
    						   // data register contains a byte which has not moved to shift reg yet
    USC1(UART0) = conf1;
}

// ------------- uart_uninit function

void uart0_uninit_tx(void) {
    uart0_disable_tx_interrupt();
}

void uart_dual_uninit_tx(void) {
    uart_dual_disable_tx_interrupt();
}

// **************************** global data (can be accessed in ISR)  ***************

// UART register definitions see esp8266_peri.h

#define DMX_DATA_BAUD		250000
#define DMX_BREAK_BAUD 	 	88000
/*
#define UART_STOP_BIT_NUM_SHIFT  4
TWO_STOP_BIT             = 0x3
ONE_STOP_BIT             = 0x1,

#define UART_BIT_NUM_SHIFT       2
EIGHT_BITS = 0x3

parity
#define UCPAE   1  //Parity Enable			(possibly set for none??)
#define UCPA    0  //Parity 0:even, 1:odd

111100 = 8n2  = 60 = 0x3C  (or 0x3E if bit1 is set for no parity)
011100 = 8n1  = 28 = 0x1C

*/

#define FORMAT_8N2			0x3C
#define FORMAT_8E1			0x1C


 //***** states indicate current position in DMX stream
    #define DMX_STATE_BREAK 0
    #define DMX_STATE_START 1
    #define DMX_STATE_DATA 2
    #define DMX_STATE_IDLE 3
	#define DMX_STATE_BREAK_SENT 4
	
	//***** interrupts to wait before changing Baud
    #define DATA_END_WAIT 50		//initially was 25 with processor at 80 mHz  set to 50 @ 160mHz
    #define BREAK_SENT_WAIT 80		//initially was 70 with processor at 80 mHz  set to 80 @ 160mHz

	//***** status is if interrupts are enabled and IO is active
    #define ISR_DISABLED 		0
    #define ISR_OUTPUT_ENABLED 	1
    #define ISR_DUAL_OUTPUT_ENABLED 	2


/*******************************************************************************
 ***********************  LX8266DMX member functions  ********************/

LX8266U0DMX::LX8266U0DMX ( void ) {
	_direction_pin = DIRECTION_PIN_NOT_USED;	//optional
	_slots = DMX_MAX_SLOTS;
	_interrupt_status = ISR_DISABLED;
	clearSlots();
}

LX8266U0DMX::~LX8266U0DMX ( void ) {
    stop();
}

void LX8266U0DMX::startOutput ( void ) {
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, HIGH);
	}
	if ( _interrupt_status != ISR_OUTPUT_ENABLED ) {
		stop();
	}
	if ( _interrupt_status == ISR_DISABLED ) {	//prevent messing up sequence if already started...
		_interrupt_status = ISR_OUTPUT_ENABLED;
		_dmx_send_state = DMX_STATE_BREAK;
		_idle_count = 0;
		uart0_init_tx(DMX_BREAK_BAUD, FORMAT_8E1, this);//starts interrupt because fifo is empty								
	}
}

void LX8266U0DMX::startDualOutput ( void ) {
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, HIGH);
	}
	if ( _interrupt_status != ISR_OUTPUT_ENABLED ) {
		stop();
	}
	if ( _interrupt_status == ISR_DISABLED ) {	//prevent messing up sequence if already started...
		_interrupt_status = ISR_DUAL_OUTPUT_ENABLED;
		_dmx_send_state = DMX_STATE_BREAK;
		_idle_count = 0;
		uart_dual_init_tx(DMX_BREAK_BAUD, FORMAT_8E1, this);//starts interrupt because fifo is empty								
	}
}

void LX8266U0DMX::stop ( void ) { 
	if ( _interrupt_status == ISR_OUTPUT_ENABLED ) {
	   uart0_uninit_tx();
	} else if ( _interrupt_status == ISR_DUAL_OUTPUT_ENABLED ) {
	   uart_dual_uninit_tx();
	}
	_interrupt_status = ISR_DISABLED;
}

void LX8266U0DMX::setDirectionPin( uint8_t pin ) {
	_direction_pin = pin;
	pinMode(_direction_pin, OUTPUT);
}

uint16_t LX8266U0DMX::numberOfSlots (void) {
	return _slots;
}

void LX8266U0DMX::setMaxSlots (int slots) {
	_slots = max(slots, DMX_MIN_SLOTS);
}

void LX8266U0DMX::setSlot (int slot, uint8_t value) {
	_dmxData[slot] = value;
}

uint8_t LX8266U0DMX::getSlot (int slot) {
	return _dmxData[slot];
}

void LX8266U0DMX::clearSlots (void) {
	memset(_dmxData, 0, DMX_MAX_SLOTS+1);
}

uint8_t* LX8266U0DMX::dmxData(void) {
	return &_dmxData[0];
}


/*!
 * @discussion TX FIFO EMPTY INTERRUPT
 *
 * this routine is called when UART fifo is empty
 *
 * what this does is to push the next byte into the fifo register
 *
 * when that byte is shifted out and the fifo is empty , the ISR is called again
 *
 * and the cycle repeats...
 *
 * until _slots worth of bytes have been sent on succesive triggers of the ISR
 *
 * and then the fifo empty interrupt is allowed to trigger 25 times to insure the last byte is fully sent
 *
 * then the break/mark after break is sent at a different speed
 *
 * and then the fifo empty interrupt is allowed to trigger 60 times to insure the MAB is fully sent
 *
 * then the baud is restored and the start code is sent
 *
 * and then on the next fifo empty interrupt
 *
 * the next data byte is sent
 *
 * and the cycle repeats...
*/

ICACHE_RAM_ATTR void LX8266U0DMX::txEmptyInterruptHandler(void) {

	switch ( _dmx_send_state ) {
		
		case DMX_STATE_BREAK:
			// set the slower baud rate and send the break
			uart0_set_baudrate(UART0, DMX_BREAK_BAUD);
			uart0_set_config(UART0, FORMAT_8E1);			
			_dmx_send_state = DMX_STATE_BREAK_SENT;
			_idle_count = 0;
			USF(0) = 0x0;
			break;		// <- DMX_STATE_BREAK
			
		case DMX_STATE_START:
			// set the baud to full speed and send the start code
			uart0_set_baudrate(UART0, DMX_DATA_BAUD);
			uart0_set_config(UART0, FORMAT_8N2);	
			_next_send_slot = 0;
			_dmx_send_state = DMX_STATE_DATA;
			USF(0) = _dmxData[_next_send_slot++];	//send next slot (start code)
			break;		// <- DMX_STATE_START
		
		case DMX_STATE_DATA:
			// send the next data byte until the end is reached
			USF(0) = _dmxData[_next_send_slot++];	//send next slot
			if ( _next_send_slot > _slots ) {
				_dmx_send_state = DMX_STATE_IDLE;
				_idle_count = 0;
			}
			break;		// <- DMX_STATE_DATA
			
		case DMX_STATE_IDLE:
			// wait a number of interrupts to be sure last data byte is sent before changing baud
			_idle_count++;
			if ( _idle_count > DATA_END_WAIT ) {
				_dmx_send_state = DMX_STATE_BREAK;
			}
			break;		// <- DMX_STATE_IDLE
			
		case DMX_STATE_BREAK_SENT:
			//wait to insure MAB before changing baud back to data speed (takes longer at slower speed)
			_idle_count++;
			if ( _idle_count > BREAK_SENT_WAIT ) {			
				_dmx_send_state = DMX_STATE_START;
			}
			break;		// <- DMX_STATE_BREAK_SENT
	}
}

