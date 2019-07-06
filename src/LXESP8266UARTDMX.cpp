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
    v2.1 - add RDM controller support
    v2.2 - RDM device support
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

#include <LXESP8266UARTDMX.h>
#include <rdm/rdm_utility.h>

LX8266DMX ESP8266DMX;

UID LX8266DMX::THIS_DEVICE_ID(0x6C, 0x78, 0x00, 0x00, 0x00, 0x01);

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
 
void uart_tx_interrupt_handler(LX8266DMX* dmxo);
void uart_rx_interrupt_handler(LX8266DMX* dmxi);
void uart_rdm_interrupt_handler(LX8266DMX* dmxr);
void uart__tx_flush(void);
void uart__rx_flush(void);
void uart_enable_rx_interrupt(LX8266DMX* dmxi);
void uart_disable_rx_interrupt(void);
void uart_enable_tx_interrupt(LX8266DMX* dmxo);
void uart_disable_tx_interrupt(void);
void uart_set_baudrate(int uart_nr, int baud_rate);

void uart_init_tx(int baudrate, byte config, LX8266DMX* dmxo);
void uart_init_rx(int baudrate, byte config, LX8266DMX* dmxi);
void uart_init_rdm(int baudrate, byte config, int txbaudrate, byte txconfig, LX8266DMX* dmxr);

void uart_uninit_tx(void);
void uart_uninit_rx(void);
void uart_uninit_rdm(void);

// ####################################################################################################

// UART register definitions see esp8266_peri.h

ICACHE_RAM_ATTR void uart_tx_interrupt_handler(LX8266DMX* dmxo) {

    // -------------- UART 1 --------------
    // check uart status register 
    // if fifo is empty clear interrupt
    // then call _tx_empty_irq
	  if(U1IS & (1 << UIFE)) {
			U1IC = (1 << UIFE);
			dmxo->txEmptyInterruptHandler();
	  }
	 
}

ICACHE_RAM_ATTR void uart_rx_interrupt_handler(LX8266DMX* dmxi) {

    // -------------- UART 0 --------------
    // check uart status register 
    // if read buffer is full, call receiveInterruptHandler and then clear interrupt

	  while (U0IS & (1 << UIFF)) {
			dmxi->byteReceived((char) (U0F & 0xff));
			U0IC |= (1 << UIFF);
	  }
     
     // if break detected, call receiveInterruptHandler and then clear interrupt
     if ( (U0IS & (1 << UIBD)) ) {				//break detected
     		dmxi->breakReceived();
     		U0IC |= (1 << UIBD);
     }
}

ICACHE_RAM_ATTR void uart_rdm_interrupt_handler(LX8266DMX* dmxr) {

    // -------------- UART 0 --------------
    // check uart status register 
    // if read buffer is full, call receiveInterruptHandler and then clear interrupt

	  while(U0IS & (1 << UIFF)) {
			dmxr->byteReceived((char) (U0F & 0xff));
			U0IC |= (1 << UIFF);
	  }
     
     // if break detected, call receiveInterruptHandler and then clear interrupt
     if ( (U0IS & (1 << UIBD)) ) {				//break detected
     		dmxr->breakReceived();
     		U0IC |= (1 << UIBD);
     }
     
     if ( dmxr->rdmTaskMode() ) {
		 if (U1IS & (1 << UIFE)) {
			U1IC = (1 << UIFE);
			dmxr->rdmTxEmptyInterruptHandler();
		 }
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


// ------------- uart_enable/disable RX functions

//LX uses uart0 for rx
void uart_enable_rx_interrupt(LX8266DMX* dmxi) {
	USIC(UART0) = 0x1ff;
	ETS_UART_INTR_ATTACH(&uart_rx_interrupt_handler, dmxi);
    USIE(UART0) |= (1 << UIFF);   //receive full
    //USIE(UART0) |= (1 << UIFR); frame error
    USIE(UART0) |= (1 << UIBD);   //break detected
    ETS_UART_INTR_ENABLE();
}

//LX uses uart0 for rx
void uart_disable_rx_interrupt(void) {
   USIE(UART0) &= ~(1 << UIFF);   //receive full
   USIE(UART0) &= ~(1 << UIBD);   //break detected
   //ETS_UART_INTR_DISABLE();		disables all UART interrupts including Hardware serial
}

// ------------- uart_enable/disable TX functions
//LX uses uart1 for tx
void uart_enable_tx_interrupt(LX8266DMX* dmxo) {
	USIC(UART1) = 0x1ff;								//clear interrupts	
	ETS_UART_INTR_ATTACH(&uart_tx_interrupt_handler, dmxo);
   USIE(UART1) |= (1 << UIFE);							//enable fifo empty interrupt
   ETS_UART_INTR_ENABLE();
}

//LX uses uart1 for tx
void uart_disable_tx_interrupt(void) {
   USIE(UART1) &= ~(1 << UIFE);
   //ETS_UART_INTR_DISABLE();		disables all UART interrupts including Hardware serial
}

// ------------- uart_enable/disable RDM functions

void uart_enable_rdm_interrupts(LX8266DMX* dmxr) {
//TX
	USIC(UART1) = 0x1ff;
	if (dmxr->rdmTaskMode()) {		//only enable if in send task mode
		USIE(UART1) |= (1 << UIFE);
	}

//RX
	USIC(UART0) = 0x1ff;
    USIE(UART0) |= (1 << UIFF);   //receive full
    USIE(UART0) |= (1 << UIBD);   //break detected
    
    ETS_UART_INTR_ATTACH(&uart_rdm_interrupt_handler, dmxr);
    ETS_UART_INTR_ENABLE();
}

void uart_disable_rdm_interrupts(void) {
//TX
   USIE(UART1) &= ~(1 << UIFE);
//RX
   USIE(UART0) &= ~(1 << UIFF);
}

// ------------- uart_set functions

//LX uses uart1 for tx, uart0 for rx
void uart_set_baudrate(int uart_nr, int baud_rate) {
    USD(uart_nr) = (ESP8266_CLOCK / baud_rate);
}

//LX uses uart1 for tx, uart0 for rx
void uart_set_config(int uart_nr, byte config) {
    USC0(uart_nr) = config;
}

// ------------- uart_init functions

void uart_init_tx(int baudrate, byte config, LX8266DMX* dmxo) {
	pinMode(2, SPECIAL);
	uint32_t conf1 = 0x00000000;
	
    uart_set_baudrate(UART1, baudrate);
    USC0(UART1) = config;
    uart_tx_flush();
    uart_enable_tx_interrupt(dmxo);

    //conf1 |= (0x00 << UCFET);// tx empty threshold is zero
    						   // tx fifo empty interrupt triggers continuously unless
    						   // data register contains a byte which has not moved to shift reg yet
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

void uart_init_rdm(int baudrate, byte config, int txbaudrate, byte txconfig, LX8266DMX* dmxr) {
//TX
	pinMode(2, SPECIAL);
	uint32_t conf1 = 0x00000000;
	
    uart_set_baudrate(UART1, txbaudrate);
    USC0(UART1) = txconfig;
    uart_tx_flush();

    //conf1 |= (0x00 << UCFET);// tx empty threshold is zero
    						   // tx fifo empty interrupt triggers continuously unless
    						   // data register contains a byte which has not moved to shift reg yet
    USC1(UART1) = conf1;

//RX
    conf1 = 0x00000000;
    pinMode(3, SPECIAL);
    uart_set_baudrate(UART0, baudrate);
    USC0(UART0) = config;
    
    conf1 |= (0x01 << UCFFT);
    USC1(UART0) = conf1;

    uart_rx_flush();
    
    uart_enable_rdm_interrupts(dmxr);
}

// ------------- uart_uninit functions

void uart_uninit_tx(void) {
    uart_disable_tx_interrupt();
	 pinMode(2, INPUT);
}

void uart_uninit_rx(void) {
    uart_disable_rx_interrupt();
	 pinMode(3, INPUT);
}

void uart_uninit_rdm(void) {
    uart_uninit_rx();
	uart_uninit_tx();
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
    #define ISR_INPUT_ENABLED 	2
    #define ISR_RDM_ENABLED 	3


/*******************************************************************************
 ***********************  LX8266DMX member functions  ********************/

LX8266DMX::LX8266DMX ( void ) {
	_direction_pin = DIRECTION_PIN_NOT_USED;	//optional
	_slots = DMX_MAX_SLOTS;
	_interrupt_status = ISR_DISABLED;
	clearSlots();
}

LX8266DMX::~LX8266DMX ( void ) {
    stop();
    _receive_callback = NULL;
}

void LX8266DMX::startOutput ( void ) {
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
		uart_init_tx(DMX_BREAK_BAUD, FORMAT_8E1, this);//starts interrupt because fifo is empty								
	}
}

void LX8266DMX::startInput ( void ) {
	if ( _direction_pin != DIRECTION_PIN_NOT_USED ) {
		digitalWrite(_direction_pin, LOW);
	}
	if ( _interrupt_status != ISR_INPUT_ENABLED ) {
		stop();
	}
	if ( _interrupt_status == ISR_DISABLED ) {	//prevent messing up sequence if already started...
	   _dmx_read_state = DMX_STATE_IDLE;
	   uart_init_rx(DMX_DATA_BAUD, FORMAT_8N2, this);
	   _interrupt_status = ISR_INPUT_ENABLED;
	}
}

void LX8266DMX::startRDM ( uint8_t pin, uint8_t direction ) {
	setDirectionPin(pin);
	_rdm_task_mode = direction;
	
	digitalWrite(_direction_pin, HIGH);

	if ( _interrupt_status != ISR_RDM_ENABLED ) {
		stop();
	}
	if ( _interrupt_status == ISR_DISABLED ) {
		_interrupt_status = ISR_RDM_ENABLED;
		//TX
		_dmx_send_state = DMX_STATE_BREAK;
		_idle_count = 0;
		//RX
		_dmx_read_state = DMX_STATE_IDLE;
	    uart_init_rdm(DMX_DATA_BAUD, FORMAT_8N2, DMX_BREAK_BAUD, FORMAT_8E1, this);							
	}
	
	if ( direction == 0 ) {
		digitalWrite(_direction_pin, LOW);
	}
}

void LX8266DMX::stop ( void ) { 
	if ( _interrupt_status == ISR_OUTPUT_ENABLED ) {
		uart_uninit_tx();
	} else if ( _interrupt_status == ISR_INPUT_ENABLED ) {
		uart_uninit_rx();
	} else if ( _interrupt_status == ISR_RDM_ENABLED ) {
		uart_uninit_rdm();
	}
	_interrupt_status = ISR_DISABLED;
}

void LX8266DMX::setDirectionPin( uint8_t pin ) {
	_direction_pin = pin;
	pinMode(_direction_pin, OUTPUT);
}

uint16_t LX8266DMX::numberOfSlots (void) {
	return _slots;
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

void LX8266DMX::clearSlots (void) {
	memset(_dmxData, 0, DMX_MAX_SLOTS+1);
}

uint8_t* LX8266DMX::dmxData(void) {
	return &_dmxData[0];
}

uint8_t* LX8266DMX::rdmData( void ) {
	return _rdmPacket;
}

uint8_t* LX8266DMX::receivedData( void ) {
	return _receivedData;
}

uint8_t* LX8266DMX::receivedRDMData( void ) {
	return _rdmData;
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

ICACHE_RAM_ATTR void LX8266DMX::txEmptyInterruptHandler(void) {

	switch ( _dmx_send_state ) {
		
		case DMX_STATE_BREAK:
			// set the slower baud rate and send the break
			uart_set_baudrate(UART1, DMX_BREAK_BAUD);
			uart_set_config(UART1, FORMAT_8E1);			
			_dmx_send_state = DMX_STATE_BREAK_SENT;
			_idle_count = 0;
			USF(1) = 0x0;
			break;		// <- DMX_STATE_BREAK
			
		case DMX_STATE_START:
			// set the baud to full speed and send the start code
			uart_set_baudrate(UART1, DMX_DATA_BAUD);
			uart_set_config(UART1, FORMAT_8N2);	
			_next_send_slot = 0;
			_dmx_send_state = DMX_STATE_DATA;			
			USF(1) = _dmxData[_next_send_slot++];	//send next slot (start code)
			break;		// <- DMX_STATE_START
		
		case DMX_STATE_DATA:
			// send the next data byte until the end is reached
			USF(1) = _dmxData[_next_send_slot++];	//send next slot
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

ICACHE_RAM_ATTR void LX8266DMX::rdmTxEmptyInterruptHandler(void) {

	if ( _rdm_task_mode == DMX_TASK_SEND_RDM ) {
		switch ( _dmx_send_state ) {
		
			case DMX_STATE_BREAK:
				// set the slower baud rate and send the break
				uart_set_baudrate(UART1, DMX_BREAK_BAUD);
				uart_set_config(UART1, FORMAT_8E1);			
				_dmx_send_state = DMX_STATE_BREAK_SENT;
				_idle_count = 0;
				USF(1) = 0x0;
				break;		// <- DMX_STATE_BREAK
			
			case DMX_STATE_START:
				// set the baud to full speed and send the start code
				uart_set_baudrate(UART1, DMX_DATA_BAUD);
				uart_set_config(UART1, FORMAT_8N2);	
				_next_send_slot = 0;
				_dmx_send_state = DMX_STATE_DATA;
				USF(1) = _rdmPacket[_next_send_slot++];	//send next slot (start code)
				break;		// <- DMX_STATE_START
		
			case DMX_STATE_DATA:
				// send the next data byte until the end is reached
				USF(1) = _rdmPacket[_next_send_slot++];	//send next slot
				if ( _next_send_slot >= _rdm_len ) {			// unlike DMX slots (which are 1 based + start code), _rdm_len is zero based, requiring >=
					_dmx_send_state = DMX_STATE_IDLE;
					_idle_count = 0;
				}
				break;		// <- DMX_STATE_DATA
			
			case DMX_STATE_IDLE:
				// wait a number of interrupts to be sure last data byte is sent before changing baud
				_idle_count++;
				if ( _idle_count > DATA_END_WAIT ) {
					_dmx_send_state = DMX_STATE_BREAK;
					
					//setTask to receive
					USIE(UART1) &= ~(1 << UIFE); 			// uart_disable_tx_interrupt();
					digitalWrite(_direction_pin, LOW);		// call from interrupt only because receiving starts
					_next_read_slot = 0;						// and these flags need to be set
					_packet_length = DMX_MAX_FRAME;			// but no bytes read from fifo until next task loop
					if ( _rdm_read_handled ) {
						_dmx_read_state = DMX_READ_STATE_RECEIVING;
					} else {
						_dmx_read_state = DMX_READ_STATE_IDLE;// if not after controller message, wait for a break
					}										  // signaling start of packet
					_rdm_task_mode = DMX_TASK_RECEIVE;
					
				}
				break;		// <- DMX_STATE_IDLE
			
			case DMX_STATE_BREAK_SENT:
				//wait to insure MAB before changing baud back to data speed (takes longer at slower speed)
				_idle_count++;
				if ( _idle_count > BREAK_SENT_WAIT ) {			
					_dmx_send_state = DMX_STATE_START;
				}
				break;		// <- DMX_STATE_BREAK_SENT
				
			}				// <- switch
		
	} else  {	// Send type state other than DMX_TASK_SEND_RDM
				// (handler not called when DMX_TASK_RECEIVE)
		switch ( _dmx_send_state ) {
		
			case DMX_STATE_BREAK:
				// set the slower baud rate and send the break
				uart_set_baudrate(UART1, DMX_BREAK_BAUD);
				uart_set_config(UART1, FORMAT_8E1);			
				_dmx_send_state = DMX_STATE_BREAK_SENT;
				_idle_count = 0;
				USF(1) = 0x0;
				break;		// <- DMX_STATE_BREAK
			
			case DMX_STATE_START:
				// set the baud to full speed and send the start code
				uart_set_baudrate(UART1, DMX_DATA_BAUD);
				uart_set_config(UART1, FORMAT_8N2);	
				_next_send_slot = 0;
				_dmx_send_state = DMX_STATE_DATA;
				USF(1) = _dmxData[_next_send_slot++];	//send next slot (start code)
				break;		// <- DMX_STATE_START
		
			case DMX_STATE_DATA:
				// send the next data byte until the end is reached
				USF(1) = _dmxData[_next_send_slot++];	//send next slot
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
					
					if ( _rdm_task_mode == DMX_TASK_SET_SEND ) {
						_rdm_task_mode = DMX_TASK_SEND;
					} else if ( _rdm_task_mode == DMX_TASK_SET_SEND_RDM ) {
						_rdm_task_mode = DMX_TASK_SEND_RDM;
					}
				}
				break;		// <- DMX_STATE_IDLE
			
			case DMX_STATE_BREAK_SENT:
				//wait to insure MAB before changing baud back to data speed (takes longer at slower speed)
				_idle_count++;
				if ( _idle_count > BREAK_SENT_WAIT ) {			
					_dmx_send_state = DMX_STATE_START;
				}
				break;		// <- DMX_STATE_BREAK_SEN
			
		}				// <- switch
	}					// <- state other than DMX_TASK_SEND_RDM
}

//************************************************************************************

void LX8266DMX::printReceivedData( void ) {
	for(int j=0; j<_next_read_slot; j++) {
		Serial.println(_receivedData[j]);
	}
}

ICACHE_RAM_ATTR void LX8266DMX::packetComplete( void ) {
	if ( _receivedData[0] == 0 ) {				//zero start code is DMX
		if ( _rdm_read_handled == 0 ) {			// not handled by specific method
			if ( _next_read_slot > DMX_MIN_SLOTS ) {
				_slots = _next_read_slot - 1;				//_next_read_slot represents next slot so subtract one
				for(int j=0; j<_next_read_slot; j++) {	//copy dmx values from read buffer
					_dmxData[j] = _receivedData[j];
				}
	
				if ( _receive_callback != NULL ) {
					_receive_callback(_slots);
				}
			}
		}
	} else {
		if ( _receivedData[0] == RDM_START_CODE ) {			//zero start code is RDM
			if ( _rdm_read_handled == 0 ) {					// not handled by specific method
				if ( validateRDMPacket(_receivedData) ) {	// evaluate checksum
					uint8_t plen = _receivedData[2] + 2;
					for(int j=0; j<plen; j++) {
						_rdmData[j] = _receivedData[j];
					}
					if ( _rdm_receive_callback != NULL ) {
						_rdm_receive_callback(plen);
					}
				}
			}
		} else {
#if defined LXESP8266UARTDMX_DEBUG
			Serial.println("________________ unknown data packet ________________");
			printReceivedData();
#endif
		}
	}
	resetFrame();
}

ICACHE_RAM_ATTR void LX8266DMX::resetFrame( void ) {		
	_dmx_read_state = DMX_READ_STATE_IDLE;						// insure wait for next break
	//_dmx_send_state????
}

ICACHE_RAM_ATTR void LX8266DMX::breakReceived( void ) {
	if ( _dmx_read_state == DMX_READ_STATE_RECEIVING ) {	// break has already been detected
		if ( _next_read_slot > 1 ) {						// break before end of maximum frame
			if ( _receivedData[0] == 0 ) {				// zero start code is DMX
				packetComplete();						// packet terminated with slots<512
			}
		}
	}
	_dmx_read_state = DMX_READ_STATE_RECEIVING;
	_next_read_slot = 0;
	_packet_length = DMX_MAX_FRAME;						// default to receive complete frame
}

ICACHE_RAM_ATTR void LX8266DMX::byteReceived(uint8_t c) {
	if ( _dmx_read_state == DMX_READ_STATE_RECEIVING ) {
		_receivedData[_next_read_slot] = c;
		if ( _next_read_slot == 2 ) {						//RDM length slot
			if ( _receivedData[0] == RDM_START_CODE ) {			//RDM start code
				if ( _rdm_read_handled == 0 ) {
					_packet_length = c + 2;				//add two bytes for checksum
				}
			} else if ( _receivedData[0] == 0xFE ) {	//RDM Discovery Response
				_packet_length = DMX_MAX_FRAME;
			} else if ( _receivedData[0] != 0 ) {		// if Not Null Start Code
				_dmx_read_state = DMX_STATE_IDLE;			//unrecognized, ignore packet
			}
		}
	
		_next_read_slot++;
		if ( _next_read_slot >= _packet_length ) {		//reached expected end of packet
			packetComplete();
		}
	}
}


uint8_t LX8266DMX::isReceiving( void ) {
	return ( _dmx_read_state == DMX_READ_STATE_RECEIVING );
}

void LX8266DMX::setDataReceivedCallback(LXRecvCallback callback) {
	_receive_callback = callback;
}

/************************************ RDM Methods **************************************/

void LX8266DMX::setRDMReceivedCallback(LXRecvCallback callback) {
	_rdm_receive_callback = callback;
}

ICACHE_RAM_ATTR uint8_t LX8266DMX::rdmTaskMode( void ) {		// applies to bidirectional RDM connection
	return _rdm_task_mode;
}

void LX8266DMX::setTaskSendDMX( void ) {		// only valid if connection started using startRDM()
	digitalWrite(_direction_pin, HIGH);
	 _rdm_task_mode = DMX_TASK_SEND;
}


ICACHE_RAM_ATTR void LX8266DMX::restoreTaskSendDMX( void ) {		// only valid if connection started using startRDM()
	digitalWrite(_direction_pin, HIGH);
	_dmx_send_state = DMX_STATE_BREAK;
	 _rdm_task_mode = DMX_TASK_SET_SEND;
	 USIE(UART1) |= (1 << UIFE);					//restore the interrupt
	 do {
	 	delay(1);
	 } while ( _rdm_task_mode != DMX_TASK_SEND );	//set to send on interrupt pass after first DMX frame sent
}

void LX8266DMX::setTaskReceive( void ) {		// only valid if connection started using startRDM()
	_next_read_slot = 0;
	_packet_length = DMX_MAX_FRAME;
    _dmx_send_state = DMX_STATE_IDLE;
    _rdm_task_mode = DMX_TASK_RECEIVE;
    _rdm_read_handled = 0;
    USIE(UART1) &= ~(1 << UIFE);				// uart_disable_tx_interrupt();
    digitalWrite(_direction_pin, LOW);
}

void LX8266DMX::sendRawRDMPacket( uint8_t len ) {		// only valid if connection started using startRDM()
	_rdm_len = len;
	// calculate checksum:  len should include 2 bytes for checksum at the end
	uint16_t checksum = rdmChecksum(_rdmPacket, _rdm_len-2);
	_rdmPacket[_rdm_len-2] = checksum >> 8;
	_rdmPacket[_rdm_len-1] = checksum & 0xFF;

	if ( _rdm_task_mode ) {						//already sending, flag to send RDM
		_rdm_task_mode = DMX_TASK_SET_SEND_RDM;
	} else {
		digitalWrite(_direction_pin, HIGH);	//possible interrupt read & cause frame error(?)
		delayMicroseconds(100);
		_dmx_send_state = DMX_STATE_BREAK;
		_rdm_task_mode = DMX_TASK_SEND_RDM;
		 //set the interrupt
		USIE(UART1) |= (1 << UIFE);
	}
	
	while ( _rdm_task_mode ) {	//wait for packet to be sent and listening to start
		delay(1);				//_rdm_task_mode is set to 0 (receive) after RDM packet is completely sent
	}
}

void  LX8266DMX::setupRDMControllerPacket(uint8_t* pdata, uint8_t msglen, uint8_t port, uint16_t subdevice) {
	pdata[RDM_IDX_START_CODE]		= RDM_START_CODE;
  	pdata[RDM_IDX_SUB_START_CODE]	= RDM_SUB_START_CODE;
  	pdata[RDM_IDX_PACKET_SIZE]		= msglen;
  	
  	// must set target UID outside this method
  	UID::copyFromUID(THIS_DEVICE_ID, pdata, RDM_IDX_SOURCE_UID);
  	
  	pdata[RDM_IDX_TRANSACTION_NUM]	= _transaction++;
  	pdata[RDM_IDX_PORT]				= port;
  	pdata[RDM_IDX_MSG_COUNT]		= 0x00;		//(always zero for controller msgs)
  	pdata[RDM_IDX_SUB_DEV_MSB] 		= subdevice >> 8;
  	pdata[RDM_IDX_SUB_DEV_LSB] 		= subdevice & 0xFF;
  	// total always 20 bytes
}

void  LX8266DMX::setupRDMDevicePacket(uint8_t* pdata, uint8_t msglen, uint8_t rtype, uint8_t msgs, uint16_t subdevice) {
	pdata[RDM_IDX_START_CODE]		= RDM_START_CODE;
  	pdata[RDM_IDX_SUB_START_CODE]	= RDM_SUB_START_CODE;
  	pdata[RDM_IDX_PACKET_SIZE]		= msglen;
  	
  	// must set target UID outside this method
  	UID::copyFromUID(THIS_DEVICE_ID, pdata, RDM_IDX_SOURCE_UID);
  	
  	pdata[RDM_IDX_TRANSACTION_NUM]	= _transaction;		//set this on read
  	pdata[RDM_IDX_RESPONSE_TYPE]	= rtype;
  	pdata[RDM_IDX_MSG_COUNT]		= msgs;
  	pdata[RDM_IDX_SUB_DEV_MSB] 		= subdevice >> 8;
  	pdata[RDM_IDX_SUB_DEV_LSB] 		= subdevice & 0xFF;
  	// total always 20 bytes
}

void  LX8266DMX::setupRDMMessageDataBlock(uint8_t* pdata, uint8_t cmdclass, uint16_t pid, uint8_t pdl) {
	pdata[RDM_IDX_CMD_CLASS] 		= cmdclass;
  	pdata[RDM_IDX_PID_MSB] 			= (pid >> 8) & 0xFF;
  	pdata[RDM_IDX_PID_LSB]			 = pid & 0xFF;
  	pdata[RDM_IDX_PARAM_DATA_LEN] 	= pdl;
  	// total always 4 bytes
}

uint8_t LX8266DMX::sendRDMDiscoveryPacket(UID lower, UID upper, UID* single) {
	uint8_t rv = RDM_NO_DISCOVERY;
	uint8_t j;
	
	//Build RDM packet
	setupRDMControllerPacket(_rdmPacket, RDM_DISC_UNIQUE_BRANCH_MSGL, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(BROADCAST_ALL_DEVICES_ID, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_DISCOVERY_COMMAND, RDM_DISC_UNIQUE_BRANCH, RDM_DISC_UNIQUE_BRANCH_PDL);
  	UID::copyFromUID(lower, _rdmPacket, 24);
  	UID::copyFromUID(upper, _rdmPacket, 30);
	
	_rdm_read_handled = 1;
	sendRawRDMPacket(RDM_DISC_UNIQUE_BRANCH_PKTL);
	delay(3);

	// any bytes read indicate response to discovery packet
	// check if a single, complete, uncorrupted packet has been received
	// otherwise, refine discovery search
	
	if ( _next_read_slot ) {
		rv = RDM_PARTIAL_DISCOVERY;
		
		// find preamble separator
		for(j=0; j<8; j++) {
			if ( _receivedData[j] == RDM_DISC_PREAMBLE_SEPARATOR ) {
				break;
			}
		}
		// 0-7 bytes preamble
		if ( j < 8 ) {
			if ( _next_read_slot == j + 17 ) { //preamble separator plus 16 byte payload
				uint8_t bindex = j + 1;
				
				//calculate checksum of 12 slots representing UID
				uint16_t checksum = rdmChecksum(&_receivedData[bindex], 12);
				
				//convert dual bytes to payload of single bytes
				uint8_t payload[8];
				for (j=0; j<8; j++) {
					payload[j] = _receivedData[bindex] & _receivedData[bindex+1];
					bindex += 2;
				}

				if ( testRDMChecksum( checksum, payload, 6 ) ) {
					//copy UID into uldata
					rv = RDM_DID_DISCOVER;
					*single = payload;
				}
			}
		}			// j<8
		
		_rdm_read_handled = 0;
		resetFrame();
	} else {
		_rdm_read_handled = 0;
	}

	restoreTaskSendDMX();
	return rv;
}

uint8_t LX8266DMX::sendRDMDiscoveryMute(UID target, uint8_t cmd) {
	uint8_t rv = 0;

	//Build RDM packet
	// total packet length 0 parameter is 24 (+cksum =26 for sendRawRDMPacket) 
	setupRDMControllerPacket(_rdmPacket, RDM_PKT_BASE_MSG_LEN, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_DISCOVERY_COMMAND, cmd, 0x00);

	_rdm_read_handled = 1;
	sendRawRDMPacket(RDM_PKT_BASE_TOTAL_LEN);
	delay(3);
	
	if ( _next_read_slot >= (RDM_PKT_BASE_TOTAL_LEN+2) ) {				//expected pdl 2 or 8
		if ( validateRDMPacket(_receivedData) ) {
			if ( _receivedData[RDM_IDX_RESPONSE_TYPE] == RDM_RESPONSE_TYPE_ACK ) {
				if ( _receivedData[RDM_IDX_CMD_CLASS] == RDM_DISC_COMMAND_RESPONSE ) {
					if ( THIS_DEVICE_ID == UID(&_receivedData[RDM_IDX_DESTINATION_UID]) ) {
						rv = 1;
					}
				}
			} else {
#if defined LXESP8266UARTDMX_DEBUG
				Serial.println("fail ACK");
#endif
			}
		} else {
#if defined LXESP8266UARTDMX_DEBUG
			Serial.println("fail validate");
#endif
		}
		
		_rdm_read_handled = 0;
		resetFrame();
	} else {
		_rdm_read_handled = 0;
	}
	
	restoreTaskSendDMX();
	return rv;
}

uint8_t LX8266DMX::sendRDMControllerPacket( void ) {
	uint8_t rv = 0;
	_rdm_read_handled = 1;
	sendRawRDMPacket(_rdmPacket[2]+2);
	delay(3);
	
	if ( _next_read_slot > 0 ) {
		if ( validateRDMPacket(_receivedData) ) {
			uint8_t plen = _receivedData[2] + 2;
			for(int rv=0; rv<plen; rv++) {
				_rdmData[rv] = _receivedData[rv];
			}
			rv = 1;
		}
		_rdm_read_handled = 0;
		resetFrame();
	} else {
		_rdm_read_handled = 0;
	}
	
	restoreTaskSendDMX();
	return rv;
}

uint8_t LX8266DMX::sendRDMControllerPacket( uint8_t* bytes, uint8_t len ) {
	for (uint8_t j=0; j<len; j++) {
		_rdmPacket[j] = bytes[j];
	}
	return sendRDMControllerPacket();
}

uint8_t LX8266DMX::sendRDMGetCommand(UID target, uint16_t pid, uint8_t* info, uint8_t len) {
	uint8_t rv = 0;
	
	//Build RDM packet
	// total packet length 0 parameter is 24 (+cksum =26 for sendRawRDMPacket) 
	setupRDMControllerPacket(_rdmPacket, RDM_PKT_BASE_MSG_LEN, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_GET_COMMAND, pid, 0x00);
	
	if ( sendRDMControllerPacket() ) {
		if ( _rdmData[RDM_IDX_RESPONSE_TYPE] == RDM_RESPONSE_TYPE_ACK ) {
			if ( _rdmData[RDM_IDX_CMD_CLASS] == RDM_GET_COMMAND_RESPONSE ) {
				if ( THIS_DEVICE_ID == UID(&_rdmData[RDM_IDX_DESTINATION_UID]) ) {
					rv = 1;
					for(int j=0; j<len; j++) {
						info[j] = _rdmData[24+j];
					}
				}
			}
		} else {
#if defined LXESP8266UARTDMX_DEBUG
			Serial.println("fail ACK");
#endif
		}
		
	} else {
#if defined LXESP8266UARTDMX_DEBUG
		Serial.println("no valid response");
#endif
	}
	
	return rv;
}

uint8_t LX8266DMX::sendRDMSetCommand(UID target, uint16_t pid, uint8_t* info, uint8_t len) {
	uint8_t rv = 0;
	
	//Build RDM packet
	// total packet length 1 byte parameter is 25 (+cksum =27 for sendRawRDMPacket) 
	setupRDMControllerPacket(_rdmPacket, RDM_PKT_BASE_MSG_LEN+len, RDM_PORT_ONE, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_SET_COMMAND, pid, len);
	for(int j=0; j<len; j++) {
		_rdmPacket[24+j] = info[j];
	}
	
	if ( sendRDMControllerPacket() ) {
		if ( _rdmData[RDM_IDX_RESPONSE_TYPE] == RDM_RESPONSE_TYPE_ACK ) {
			if ( _rdmData[RDM_IDX_CMD_CLASS] == RDM_SET_COMMAND_RESPONSE ) {
				if ( THIS_DEVICE_ID == UID(&_rdmData[RDM_IDX_DESTINATION_UID]) ) {
					rv = 1;
				}
			}
		} else {
#if defined LXESP8266UARTDMX_DEBUG
			Serial.println("fail ACK");
#endif
		}
	} else {
#if defined LXESP8266UARTDMX_DEBUG
		Serial.println("no valid response");
#endif
	}
	
	return rv;
}

void LX8266DMX::sendRDMGetResponse(UID target, uint16_t pid, uint8_t* info, uint8_t len) {
	uint8_t plen = RDM_PKT_BASE_MSG_LEN+len;
	
	//Build RDM packet
	setupRDMDevicePacket(_rdmPacket, plen, RDM_RESPONSE_TYPE_ACK, 0, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, RDM_GET_COMMAND_RESPONSE, pid, len);
	for(int j=0; j<len; j++) {
		_rdmPacket[24+j] = info[j];
	}
	
	sendRawRDMPacket(plen+2);	//add 2 bytes for checksum
}

void LX8266DMX::sendAckRDMResponse(uint8_t cmdclass, UID target, uint16_t pid) {
	uint8_t plen = RDM_PKT_BASE_MSG_LEN;
	
	//Build RDM packet
	setupRDMDevicePacket(_rdmPacket, plen, RDM_RESPONSE_TYPE_ACK, 0, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, cmdclass, pid, 0x00);
	
	sendRawRDMPacket(plen+2);	//add 2 bytes for checksum
}

void LX8266DMX::sendMuteAckRDMResponse(uint8_t cmdclass, UID target, uint16_t pid) {
	uint8_t plen = RDM_PKT_BASE_MSG_LEN + 2;
	
	//Build RDM packet
	setupRDMDevicePacket(_rdmPacket, plen, RDM_RESPONSE_TYPE_ACK, 0, RDM_ROOT_DEVICE);
	UID::copyFromUID(target, _rdmPacket, 3);
	setupRDMMessageDataBlock(_rdmPacket, cmdclass, pid, 0x02);
	
	sendRawRDMPacket(plen+2);	//add 2 bytes for checksum
}

void LX8266DMX::sendRDMDiscoverBranchResponse( void ) {
	// should be listening when this is called
	
	_rdmPacket[0] = 0;
	_rdmPacket[1] = 0xFE;
	_rdmPacket[2] = 0xFE;
	_rdmPacket[3] = 0xFE;
	_rdmPacket[4] = 0xFE;
	_rdmPacket[5] = 0xFE;
	_rdmPacket[6] = 0xFE;
	_rdmPacket[7] = 0xFE;
	_rdmPacket[8] = 0xAA;
	
	_rdmPacket[9] = THIS_DEVICE_ID.rawbytes()[0] | 0xAA;
	_rdmPacket[10] = THIS_DEVICE_ID.rawbytes()[0] | 0x55;
	_rdmPacket[11] = THIS_DEVICE_ID.rawbytes()[1] | 0xAA;
	_rdmPacket[12] = THIS_DEVICE_ID.rawbytes()[1] | 0x55;
	
	_rdmPacket[13] = THIS_DEVICE_ID.rawbytes()[2] | 0xAA;
	_rdmPacket[14] = THIS_DEVICE_ID.rawbytes()[2] | 0x55;
	_rdmPacket[15] = THIS_DEVICE_ID.rawbytes()[3] | 0xAA;
	_rdmPacket[16] = THIS_DEVICE_ID.rawbytes()[3] | 0x55;
	_rdmPacket[17] = THIS_DEVICE_ID.rawbytes()[4] | 0xAA;
	_rdmPacket[18] = THIS_DEVICE_ID.rawbytes()[4] | 0x55;
	_rdmPacket[19] = THIS_DEVICE_ID.rawbytes()[5] | 0xAA;
	_rdmPacket[20] = THIS_DEVICE_ID.rawbytes()[5] | 0x55;
	
	uint16_t checksum = rdmChecksum(&_rdmPacket[9], 12);
	uint8_t bite = checksum >> 8;
	_rdmPacket[21] = bite | 0xAA;
	_rdmPacket[22] = bite | 0x55;
	bite = checksum & 0xFF;
	_rdmPacket[23] = bite | 0xAA;
	_rdmPacket[24] = bite | 0x55;
	
	// send (no break)
	_rdm_len = 25;
	digitalWrite(_direction_pin, HIGH); 	// could cut off receiving (?)
	delayMicroseconds(100);
	_next_send_slot = 1;//SKIP start code
	uart_set_baudrate(UART1, DMX_DATA_BAUD);
	uart_set_config(UART1, FORMAT_8N2);	
	_dmx_send_state = DMX_STATE_DATA;
	
	_rdm_task_mode = DMX_TASK_SEND_RDM;
	 //set the interrupt
	USIE(UART1) |= (1 << UIFE);

	
	while ( _rdm_task_mode ) {	//wait for packet to be sent and listening to start again
		delay(1);				//_rdm_task_mode is set to 0 (receive) after RDM packet is completely sent
	}
}
