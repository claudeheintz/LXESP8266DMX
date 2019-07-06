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
RX (3) |----------------------| 1      Vcc 8 |------(+5v)
       |                      |              |              DMX Output
       |                 +----| 2 !RE    B 7 |---------------- Pin 2
       |                 |    |              |
 (4/5) |----{+5v/GND}----+----| 3 DE     A 6 |---------------- Pin 3
       |                      |              |
TX (2) |----------------------| 4 DI   Gnd 5 |---+------------ Pin 1
       |                                         |
       |                                       (GND)

       Data Enable (DE) and Inverted Read Enable (!RE) can be wired to +5v for output or Gnd for input
       if direction switching is not needed.

--------------------------------------------------------------------------
------------------------  About UART GPIOs  ------------------------------

  UART0 is used by Serial for debug output (also see datasheet pg 21)
        so UART1 is used for DMX output (tx)
  UART1 does not have the rx pin available on most ESP modules so
        UART0 is used for DMX input (rx)


 UART0 TX: 1 or 2				SPECIAL or FUNCTION_4
 UART0 RX: 3						SPECIAL

 UART0 SWAP TX: 15
 UART0 SWAP RX: 13

 UART1 TX: 2 or 7 (NC)		SPECIAL or FUNCTION_4
 UART1 RX: 8 (NC)				FUNCTION_4

 UART1 SWAP TX: 11 (NC)
 UART1 SWAP RX: 6 (NC)

 NC = Not Connected to Module Pads --> No Access
      Pins 6-11 typically connected to flash on most modules
      see http://arduino.esp8266.com/versions/1.6.5-1160-gef26c5f/doc/reference.html
 */


#ifndef LX8266DMX_H
#define LX8266DMX_H


#include <Arduino.h>
#include <inttypes.h>
#include <rdm/UID.h>

#define DMX_MIN_SLOTS 24
#define RDM_MAX_FRAME 257
#define DMX_MAX_SLOTS 512
#define DMX_MAX_FRAME 513

 //***** states indicate current position in DMX stream
    #define DMX_READ_STATE_IDLE 0
    #define DMX_READ_STATE_RECEIVING 1
    
#define DMX_TASK_RECEIVE		0   
#define DMX_TASK_SEND			1
#define DMX_TASK_SEND_RDM		2
#define DMX_TASK_SET_SEND		3
#define DMX_TASK_SET_SEND_RDM	4

#define RDM_NO_DISCOVERY		0
#define RDM_PARTIAL_DISCOVERY	1
#define RDM_DID_DISCOVER		2

#define DIRECTION_PIN_NOT_USED 255

typedef void (*LXRecvCallback)(int);

/*!   
@class LX8266DMX
@abstract
   LX8266DMX is a driver for sending or receiving DMX using an ESP8266
   UART1 which has TX mapped to GPIO2 is used for output
   UART0 which is mapped to the RX pin is used for input
   
   LX8266DMX output mode continuously sends DMX once its interrupts have been enabled using startOutput().
   Use setSlot() to set the level value for a particular DMX dimmer/address/channel.
   
   LX8266DMX input mode receives DMX using the ESP8266's UART0 RX pin
   LX8266DMX continuously updates its DMX buffer once its interrupts have been enabled using startInput()
   and DMX data is received by UART0.
   Use getSlot() to read the level value for a particular DMX dimmer/address/channel.
   
   LX8266DMX is used with a single instance called ESP8266DMX.
   
   LX8266DMX is NOT compatible with Serial.begin when DMX will be read.
*/

class LX8266DMX {

  public:
  
	LX8266DMX ( void );
   ~LX8266DMX( void );
    
   /*!
    * @brief starts interrupt that continuously sends DMX output
    * @discussion Sets up baud rate, bits and parity, 
    *             sets globals accessed in ISR, 
    *             enables transmission and tx interrupt.
   */
   void startOutput( void );
   
   /*!
    * @brief starts interrupt that continuously reads DMX data
    * @discussion sets up baud rate, bits and parity, 
    *             sets globals accessed in ISR, 
    *             enables transmission and tx interrupt
   */
   void startInput( void );
   
   
   /*!
    * @brief starts interrupt that handled send/receive of RDM & DMX
    * @discussion  
   */
   void startRDM ( uint8_t pin, uint8_t direction=1 );
   
   /*!
    * @brief disables transmission and tx interrupt
   */
	void stop( void );
	
	/*!
	 * @brief optional utility sets the pin used to control driver chip's
	 *        DE (data enable) line, HIGH for output, LOW for input.     
    * @param pin to be automatically set for input/output direction
    */
   void setDirectionPin( uint8_t pin );
   
   /*!
    * @brief the current number of slots
   */
   uint16_t numberOfSlots (void);
	
	/*!
	 * @brief Sets the number of slots (aka addresses or channels) sent per DMX frame.
	 * @discussion defaults to 512 or DMX_MAX_SLOTS and should be no less DMX_MIN_SLOTS slots.  
	 *             The DMX standard specifies min break to break time no less than 1024 usecs.  
	 *             At 44 usecs per slot ~= 24
	 * @param slot the highest slot number (~24 to 512)
	*/
	void setMaxSlots (int slot);
	
	/*!
    * @brief reads the value of a slot/address/channel
    * @discussion NOTE: Data is not double buffered.  
    *                   So a complete single frame is not guaranteed.  
    *                   The ISR continuously reads the next frame into the buffer
    * @return level (0-255)
   */
   uint8_t getSlot (int slot);
   
   /*!
	 * @brief Sets the output value of a slot  Note:slot[0] is DMX start code!
	 * @param slot number of the slot aka address or channel (1-512)
	 * @param value level (0-255)
	*/
   void setSlot (int slot, uint8_t value);
   
   /*!
    * @brief zero buffer including slot[0] which is start code
   */
   void clearSlots (void);
   
   /*!
    * @brief provides direct access to data array
    * @return pointer to dmx array
   */
	uint8_t* dmxData(void);
   
	uint8_t* rdmData( void );

	uint8_t* receivedData( void );

	uint8_t* receivedRDMData( void );
   
   /*!
    * @brief UART tx empty interrupt handler
   */
  	void		txEmptyInterruptHandler(void);
  	
  	/*!
    * @brief UART RDM tx empty interrupt handler
   */
  	void		rdmTxEmptyInterruptHandler(void);
  	
  	/*!
    * @brief Function called when DMX frame has been read
    * @discussion Sets a pointer to a function that is called
    *             on the break after a DMX frame has been received.  
    *             Whatever happens in this function should be quick!  
    *             Best used to set a flag that is polled outside of ISR for available data.
   */
   void setDataReceivedCallback(LXRecvCallback callback);
   
   
   /*!
    * @brief utility for debugging prints received data
   */
   void printReceivedData( void );
   
   /*!
    * @brief called when a packet is finished being received either through start of next packet or size reached
   */
   void packetComplete( void );
   
   /*!
    * @brief sets read state to wait for break
   */
   void resetFrame( void );
   
   /*!
    * @brief called when a break is detected
   */
  	void breakReceived( void );
  	
  	/*!
    * @brief called from isr when a byte is read from register
   */
  	void byteReceived(uint8_t c);
  	
  	/*!
    * @brief returns true if _dmx_read_state == DMX_READ_STATE_RECEIVING
   */
  	uint8_t isReceiving( void );
  	
  	/************************************ RDM Methods ***********************************/
  	 
   /*!
    * @brief Function called when RDM frame has been read
    * @discussion Sets a pointer to a function that is called
    *             after an RDM frame has been received.  
    *             Whatever happens in this function should be quick!  
    *             Best used to set a flag that is polled outside of ISR for available data.
    */
   void setRDMReceivedCallback(LXRecvCallback callback);
   
   /*!
    * @brief indicate if dmx frame should be sent by bi-directional task loop
    * @discussion should only be called by task loop
    * @return 1 if dmx frame should be sent
    *  return 2 if RDM should be sent
    *  return 3 if RDM should be sent and set mode to 1 after first frame finished
    */
	uint8_t rdmTaskMode( void );
	
	
   /*!
    * @brief sets rdm task to send mode and the direction pin to HIGH
	*/
	void setTaskSendDMX( void );
	
   /*!
    * @brief sets rdm task to send mode after task mode loops.
    *        Sent after sending RDM message so DMX is resumed.
    *        Blocks until task loop sets mode to send.
	*/
	void restoreTaskSendDMX( void );
	
   /*!
    * @brief sets rdm task to receive mode
    *        Prepares variables to receive starting with next break.
    *        Sets the direction pin to LOW.
	*/
	void setTaskReceive( void );
	
   /*!
    * @brief length of the rdm packet awaiting being sent
	*/
	uint8_t rdmPacketLength( void );
	
   /*!
    * @brief sends packet using bytes from _rdmPacket ( rdmData() )
    * @discussion sets rdm task mode to DMX_TASK_SEND_RDM which causes
    *             _rdmPacket to be sent on next opportunity from task loop.
    *             after _rdmPacket is sent, task mode switches to listen for response.
    *
    *             set _rdm_read_handled flag prior to calling sendRawRDMPacket
    *             _rdm_read_handled = 1 if reading is handled by calling function
    *             _rdm_read_handled = 0 if desired to resume passive listening for next break
    */
	void sendRawRDMPacket( uint8_t len );
	
   /*!
    * @brief convenience method for setting fields in the top 20 bytes of an RDM message
    *        that will be sent.
    *        Destination UID needs to be set outside this method.
    *        Source UID is set to static member THIS_DEVICE_ID
	*/
	void setupRDMControllerPacket(uint8_t* pdata, uint8_t msglen, uint8_t port, uint16_t subdevice);
	
   /*!
    * @brief convenience method for setting fields in the top 20 bytes of an RDM message
    *        that will be sent.
    *        Destination UID needs to be set outside this method.
    *        Source UID is set to static member THIS_DEVICE_ID
	*/
	void  setupRDMDevicePacket(uint8_t* pdata, uint8_t msglen, uint8_t rtype, uint8_t msgs, uint16_t subdevice);
	
   /*!
    * @brief convenience method for setting fields in the top bytes 20-23 of an RDM message
    *        that will be sent.
	*/
	void setupRDMMessageDataBlock(uint8_t* pdata, uint8_t cmdclass, uint16_t pid, uint8_t pdl);
	
   /*!
    * @brief send discovery packet using upper and lower bounds
	* @discussion Assumes that regular DMX was sending when method is called and 
	*             so restores sending, waiting for a frame to be sent before returning.
    * @return 1 if discovered, 2 if valid packet (UID stored in uldata[12-17])
    */
    uint8_t sendRDMDiscoveryPacket(UID lower, UID upper, UID* single);
    
   /*!
    * @brief send discovery mute/un-mute packet to target UID
	* @discussion Assumes that regular DMX was sending when method is called and 
	*             so restores sending, waiting for a frame to be sent before returning.
    * @return 1 if ack response is received.
    */
    uint8_t sendRDMDiscoveryMute(UID target, uint8_t cmd);
    
   /*!
    * @brief send previously built packet in _rdmPacket and validate response
    * @discussion Response to packet, if valid, is copied into _rdmData and 1 is returned
    *             Otherwise, 0 is returned.
    */
    uint8_t sendRDMControllerPacket( void );
    
   /*!
    * @brief copies len of bytes into _rdmPacket and sends it
    * @discussion Response to packet, if valid, is copied into _rdmData and 1 is returned
    *             Otherwise, 0 is returned.
    */
    uint8_t sendRDMControllerPacket( uint8_t* bytes, uint8_t len );
    
   /*!
    * @brief send RDM_GET_COMMAND packet
	* @discussion Assumes that regular DMX was sending when method is called and 
	*             so restores sending, waiting for a frame to be sent before returning.
    * @return 1 if ack is received.
    */
    uint8_t sendRDMGetCommand(UID target, uint16_t pid, uint8_t* info, uint8_t len);
    
   /*!
    * @brief send RDM_SET_COMMAND packet
	* @discussion Assumes that regular DMX was sending when method is called and 
	*             so restores sending, waiting for a frame to be sent before returning.
    * @return 1 if ack is received.
    */
    uint8_t sendRDMSetCommand(UID target, uint16_t pid, uint8_t* info, uint8_t len);
    
     
    /*!
    * @brief send RDM_GET_COMMAND_RESPONSE with RDM_RESPONSE_TYPE_ACK
	* @discussion sends data (info) of length (len)
    */
    void sendRDMGetResponse(UID target, uint16_t pid, uint8_t* info, uint8_t len);
    
    /*!
    * @brief send RDM_SET_COMMAND_RESPONSE/RDM_DISC_COMMAND_RESPONSE with RDM_RESPONSE_TYPE_ACK
	* @discussion PDL is zero
    */
    void sendAckRDMResponse(uint8_t cmdclass, UID target, uint16_t pid);
    
    
    
    
    void sendMuteAckRDMResponse(uint8_t cmdclass, UID target, uint16_t pid);
    
    /*!
    * @brief send response to RDM_DISC_UNIQUE_BRANCH packet
    */
    void sendRDMDiscoverBranchResponse( void );
    
    /*!
    * @brief static member containing UID of this device
    * @discussion call LX8266DMX::THIS_DEVICE_ID.setBytes() or
    *                  ESP8266DMX.THIS_DEVICE_ID.setBytes() to change default
    */
    static UID THIS_DEVICE_ID;
    
  private:

	/*!
	 * @brief represents phase of sending dmx packet data/break/etc used to change baud settings
	 */
  	uint8_t  _dmx_send_state;
  	
	/*!
	 * @brief represents phase of sending dmx packet data/break/etc used to change baud settings
	 */
  	volatile uint8_t  _dmx_read_state;
  	
	/*!
	 * @brief true when ISR is enabled
	 */
  	uint8_t  _interrupt_status;
  	
	/*!
	 * @brief count of idle interrupts
	 */
  	uint8_t  _idle_count;
  	
	/*!
	 * @brief flag indicating RDM task should send dmx slots
	 */
  	uint8_t  _rdm_task_mode;
  	
	/*!
	 * @brief flag indicating RDM task should send dmx slots
	 */
  	uint8_t  _rdm_read_handled;
  	
  	/*!
	 * @brief transaction number
	 */
  	uint8_t _transaction;
  	
  	/*!
	 * @brief maximum expected length of packet
	 */
  	uint16_t  _packet_length;
  	
	/*!
	 * @brief pin used to control direction of output driver chip
	 */
  	uint8_t _direction_pin;
  	
	/*!
	 * @brief slot index indicating position of byte to be sent
	 */
  	uint16_t  _next_send_slot;
  	
	/*!
	 * @brief slot index indicating position of last byte received
	 */
  	volatile uint16_t  _next_read_slot;
  	
	/*!
	 * @brief number of dmx slots ~24 to 512
	 */
  	volatile uint16_t  _slots;
  	
	/*!
	 * @brief outgoing rdm packet length
	 */
	uint16_t  _rdm_len;
  	
	/*!
	 * @brief Array of dmx data including start code
	 */
  	uint8_t  _dmxData[DMX_MAX_FRAME];
  	
	/*!
	 * @brief Array of received bytes first byte is start code
	 */
  	uint8_t  _receivedData[DMX_MAX_FRAME];
  	
  	/*!
	 * @brief Array representing an rdm packet to be sent
	 */
	uint8_t  _rdmPacket[RDM_MAX_FRAME];
	
	/*!
	 * @brief Array representing a received rdm packet
	 */
	uint8_t  _rdmData[RDM_MAX_FRAME];
  	
   /*!
    * @brief Pointer to receive callback function
	*/
  	LXRecvCallback _receive_callback;
  	
   /*!
    * @brief Pointer to receive callback function
    */
  	LXRecvCallback _rdm_receive_callback;
  	
  	
};

extern LX8266DMX ESP8266DMX;

#endif // ifndef LX8266DMX_H