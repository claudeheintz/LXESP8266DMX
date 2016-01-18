/**************************************************************************/
/*!
    @file     LXESP8266UARTDMX.cpp
    @author   Claude Heintz
    @license  BSD (see LXESP8266UARTDMX.h)
    @copyright 2015-2016 by Claude Heintz

    DMX Driver for ESP8266 using UART1.

    @section  HISTORY

    v1.0 - First release
    v1.1 - Consolidated Output and Input into a single class
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

#include "LXESP8266UARTDMX.h"

LX8266DMX ESP8266DMX;

/* ***************** Utility functions derived from ESP HardwareSerial.cpp  ****************
   HardwareSerial.cpp - esp8266 UART support - Copyright (c) 2014 Ivan Grokhotkov. All rights reserved.
   HardwareSerial is licensed under GNU Lesser General Public License
   HardwareSerial is included in the esp8266 by ESP8266 Community board package for Arduino IDE
*/

static const int UART0 = 0;
static const int UART1 = 1;
static const int UART_NO = -1;


/**
 *  UART GPIOs
 *
 * UART0 TX: 1 or 2
 * UART0 RX: 3
 *
 * UART0 SWAP TX: 15
 * UART0 SWAP RX: 13
 *
 *
 * UART1 TX: 7 (NC) or 2
 * UART1 RX: 8 (NC)
 *
 * UART1 SWAP TX: 11 (NC)
 * UART1 SWAP RX: 6 (NC)
 *
 * NC = Not Connected to Module Pads --> No Access
 *
 */
 
void uart_tx_interrupt_handler(LX8266DMX* dmxo);
void uart_rx_interrupt_handler(LX8266DMX* dmxi);
void uart__tx_flush(void);
void uart__rx_flush(void);
void uart_enable_rx_interrupt(LX8266DMX* dmxi);
void uart_disable_rx_interrupt(void);
void uart_enable_tx_interrupt(LX8266DMX* dmxo);
void uart_disable_tx_interrupt(void);
void uart_set_baudrate(int uart_nr, int baud_rate);

void uart_init_tx(int baudrate, byte config, LX8266DMX* dmxo);
void uart_init_rx(int baudrate, byte config, LX8266DMX* dmxi);
void uart_uninit(uart_t* uart);

// ####################################################################################################

// UART register definitions see esp8266_peri.h

void ICACHE_RAM_ATTR uart_tx_interrupt_handler(LX8266DMX* dmxo) {

    // -------------- UART 1 --------------
    // check uart status register 
    // if fifo is empty clear interrupt
    // then call _tx_empty_irq
	  if(U1IS & (1 << UIFE)) {
			U1IC = (1 << UIFE);
			dmxo->txEmptyInterruptHandler();
	  }
	 
}

void ICACHE_RAM_ATTR uart_rx_interrupt_handler(LX8266DMX* dmxi) {

    // -------------- UART 0 --------------
    // check uart status register 
    // if read buffer is full, call receiveInterruptHandler and then clear interrupt

	  while(U0IS & (1 << UIFF)) {
			dmxi->receiveInterruptHandler((char) (U0F & 0xff));
			U0IC = (1 << UIFF);
	  }
     
     // if break detected, call receiveInterruptHandler and then clear interrupt
     if ( (U0IS & (1 << UIBD)) ) {				//break detected
     		dmxi->receiveInterruptHandler(0);
     		U0IC |= (1 << UIBD);
     }
}


// ####################################################################################################

//LX uses uart1 for tx
void uart_tx_flush(void) {
    uint32_t tmp = 0x00000000;

    tmp |= (1 << UCTXRST);
    
    USC0(UART1) |= (tmp);
    USC0(UART1) &= ~(tmp);
}

//LX uses uart0 for rx
void uart_rx_flush(void) {
    uint32_t tmp = 0x00000000;

    tmp |= (1 << UCRXRST);

    USC0(UART0) |= (tmp);
    USC0(UART0) &= ~(tmp);
}


//LX uses uart0 for rx
void uart_enable_rx_interrupt(LX8266DMX* dmxi) {
	USIC(UART0) = 0x1ff;
	uint8_t* uart;
	ETS_UART_INTR_ATTACH(&uart_rx_interrupt_handler, dmxi);
    USIE(UART0) |= (1 << UIFF);   //receive full
    //USIE(UART0) |= (1 << UIFR); frame error
    USIE(UART0) |= (1 << UIBD);   //break detected
    ETS_UART_INTR_ENABLE();
}

//LX uses uart0 for rx
void uart_disable_rx_interrupt(void) {
   USIE(UART0) &= ~(1 << UIFF);
   //ETS_UART_INTR_DISABLE();		disables all UART interrupts including Hardware serial
}

//LX uses uart1 for tx
void uart_enable_tx_interrupt(LX8266DMX* dmxo) {
	USIC(UART1) = 0x1ff;
	uint8_t* uart;
	ETS_UART_INTR_ATTACH(&uart_tx_interrupt_handler, dmxo);
   USIE(UART1) |= (1 << UIFE);
   ETS_UART_INTR_ENABLE();
}

//LX uses uart1 for tx
void uart_disable_tx_interrupt(void) {
   USIE(UART1) &= ~(1 << UIFE);
   //ETS_UART_INTR_DISABLE();		disables all UART interrupts including Hardware serial
}

//LX uses uart1 for tx, uart0 for rx
void uart_set_baudrate(int uart_nr, int baud_rate) {
    USD(uart_nr) = (ESP8266_CLOCK / baud_rate);
}

//LX uses uart1 for tx, uart0 for rx
void uart_set_config(int uart_nr, byte config) {
    USC0(uart_nr) = config;
}


void uart_init_tx(int baudrate, byte config, LX8266DMX* dmxo) {
	pinMode(2, SPECIAL);
	uint32_t conf1 = 0x00000000;
	
   uart_set_baudrate(UART1, baudrate);
    USC0(UART1) = config;
    uart_tx_flush();
    uart_enable_tx_interrupt(dmxo);

    conf1 |= (0x01 << UCFFT);//empty threshold 0x20
    USC1(UART1) = conf1;
}


void uart_init_rx(int baudrate, byte config, LX8266DMX* dmxi) {

    uint32_t conf1 = 0x00000000;
    pinMode(3, SPECIAL);
    uart_set_baudrate(UART0, baudrate);
    USC0(UART0) = config;
    
    conf1 |= (0x01 << UCFFT);
    USC1(UART0) = conf1;

    uart_rx_flush();
    uart_enable_rx_interrupt(dmxi);
}

void uart_uninit_tx(void) {
    uart_disable_tx_interrupt();
	 pinMode(2, INPUT);
}

void uart_uninit_rx(void) {
    uart_disable_tx_interrupt();
	 pinMode(3, INPUT);
}

// **************************** global data (can be accessed in ISR)  ***************

// UART register definitions see esp8266_peri.h

#define DMX_DATA_BAUD		250000
#define DMX_BREAK_BAUD 	 	90000
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
    #define DATA_END_WAIT 25		//initially was 25 may not be quite long enough?
    #define BREAK_SENT_WAIT 70

	//***** status is if interrupts are enabled and IO is active
    #define ISR_DISABLED 			0
    #define ISR_OUTPUT_ENABLED 	1
    #define ISR_INPUT_ENABLED 	2


/*******************************************************************************
 ***********************  LX8266DMX member functions  ********************/

LX8266DMX::LX8266DMX ( void ) {
	_direction_pin = DIRECTION_PIN_NOT_USED;	//optional
	_slots = DMX_MAX_SLOTS;
	_interrupt_status = ISR_DISABLED;
	
	//zero buffer including _dmxData[0] which is start code
    for (int n=0; n<DMX_MAX_SLOTS+1; n++) {
    	_dmxData[n] = 0;
    }
}

LX8266DMX::~LX8266DMX ( void ) {
    stop();
    _receive_callback = NULL;
}

void LX8266DMX::startOutput ( void ) {
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, HIGH);
	}
	if ( _interrupt_status == ISR_INPUT_ENABLED ) {
		stop();
	}
	if ( _interrupt_status == ISR_DISABLED ) {	//prevent messing up sequence if already started...
		_interrupt_status = ISR_OUTPUT_ENABLED;
		_dmx_state = DMX_STATE_IDLE;
		_idle_count = 0;
		uart_init_tx(DMX_DATA_BAUD, FORMAT_8N2, this);//starts interrupt because fifo is empty
		//USF(1) = 0x0;									
	}
}

void LX8266DMX::startInput ( void ) {
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, LOW);
	}
	if ( _interrupt_status == ISR_OUTPUT_ENABLED ) {
		stop();
	}
	if ( _interrupt_status == ISR_DISABLED ) {	//prevent messing up sequence if already started...
	   _dmx_state = DMX_STATE_IDLE;
	   uart_init_rx(DMX_DATA_BAUD, FORMAT_8N2, this);
		_interrupt_status = ISR_INPUT_ENABLED;
	}
}

void LX8266DMX::stop ( void ) { 
	if ( _interrupt_status == ISR_OUTPUT_ENABLED ) {
		uart_uninit_tx();
	} else if ( _interrupt_status == ISR_INPUT_ENABLED ) {
		uart_uninit_rx();
	}
	_interrupt_status = ISR_DISABLED;
}

void LX8266DMX::setDirectionPin( uint8_t pin ) {
	_direction_pin = pin;
	pinMode(_direction_pin, OUTPUT);
}

void LX8266DMX::setMaxSlots (int slots) {
	_slots = max(slots, DMX_MIN_SLOTS);
}

void LX8266DMX::setSlot (int slot, uint8_t value) {
	_dmxData[slot] = value;
}

uint8_t LX8266DMX::getSlot (int slot) {
	return _dmxData[slot];
}

uint8_t* LX8266DMX::dmxData(void) {
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

void ICACHE_RAM_ATTR LX8266DMX::txEmptyInterruptHandler(void) {

	switch ( _dmx_state ) {
		
		case DMX_STATE_BREAK:
			// set the slower baud rate and send the break
			uart_set_baudrate(UART1, DMX_BREAK_BAUD);
			uart_set_config(UART1, FORMAT_8E1);			
			_dmx_state = DMX_STATE_BREAK_SENT;
			_idle_count = 0;
			USF(1) = 0x0;
			break;		// <- DMX_STATE_BREAK
			
		case DMX_STATE_START:
			// set the baud to full speed and send the start code
			uart_set_baudrate(UART1, DMX_DATA_BAUD);
			uart_set_config(UART1, FORMAT_8N2);	
			_current_slot = 0;
			_dmx_state = DMX_STATE_DATA;
			USF(1) = _dmxData[_current_slot++];	//send next slot (start code)
			break;		// <- DMX_STATE_START
		
		case DMX_STATE_DATA:
			// send the next data byte until the end is reached
			USF(1) = _dmxData[_current_slot++];	//send next slot
			if ( _current_slot > _slots ) {
				_dmx_state = DMX_STATE_IDLE;
				_idle_count = 0;
			}
			break;		// <- DMX_STATE_DATA
			
		case DMX_STATE_IDLE:
			// wait a number of interrupts to be sure last data byte is sent before changing baud
			_idle_count++;
			if ( _idle_count > DATA_END_WAIT ) {
				_dmx_state = DMX_STATE_BREAK;
			}
			break;		// <- DMX_STATE_IDLE
			
		case DMX_STATE_BREAK_SENT:
			//wait to insure MAB before changing baud back to data speed (takes longer at slower speed)
			_idle_count++;
			if ( _idle_count > BREAK_SENT_WAIT ) {			
				_dmx_state = DMX_STATE_START;
			}
			break;		// <- DMX_STATE_BREAK_SENT
	}
}

//************************************************************************************
// WARNING:  the input portion of this library is a draft and is not tested or complete in this version

void LX8266DMX::setDataReceivedCallback(LXRecvCallback callback) {
	_receive_callback = callback;
}

/*!
 * @discussion RX ISR (receive interrupt service routine)
 *
 * this routine is called when USART receives data
 *
 * wait for break:  if have previously read data call callback function
 *
 * then on next receive:  check start code
 *
 * then on next receive:  read data until done (in which case idle)
 *
 *  NOTE: data is not double buffered
 *
 *  so a complete single frame is not guaranteed
 *
 *  the ISR will continue to read the next frame into the buffer
*/
void LX8266DMX::receiveInterruptHandler(uint8_t incoming_byte) {
	
	if ( (U0IS & (1 << UIBD)) ) {				//break detected
		_dmx_state = DMX_STATE_BREAK;
		if ( _current_slot > 0 ) {
			if ( _receive_callback != NULL ) {
				_receive_callback(_current_slot);
			}
		}
		_current_slot = 0;
		return;
	}
	
	switch ( _dmx_state ) {
	
		case DMX_STATE_BREAK:
			if ( incoming_byte == 0 ) {						//start code == zero (DMX)
				_dmx_state = DMX_STATE_DATA;
				_current_slot = 1;
			} else {
				_dmx_state = DMX_STATE_IDLE;
			}
			break;
			
		case DMX_STATE_DATA:
			_dmxData[_current_slot++] = incoming_byte;
			if ( _current_slot > DMX_MAX_SLOTS ) {
				_dmx_state = DMX_STATE_IDLE;			// go to idle, wait for next break
			}
			break;
	}
}