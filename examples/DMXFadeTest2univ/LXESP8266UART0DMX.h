/**************************************************************************/
/*!
    @file     LXESP8266UART0DMX.h
    @author   Claude Heintz
    @license  BSD (see LXESP8266UARTDMX.h)
    @copyright 2015-2017 by Claude Heintz

    DMX Output Driver for ESP8266 using UART0.
*/
/**************************************************************************/



#ifndef LX8266U0DMX_H
#define LX8266U0DMX_H

#include <Arduino.h>
#include <inttypes.h>

#define DMX_MIN_SLOTS 24
#define DMX_MAX_SLOTS 512
#define DMX_MAX_FRAME 513

#define DIRECTION_PIN_NOT_USED 255

/*!   
@class LX8266U0DMX
@abstract
   LX8266U0DMX is an output driver for sending DMX using an ESP8266
   UART0 which has TX mapped to GPIO1.
   
   LX8266U0DMX output mode continuously sends DMX once its interrupts have been enabled using startOutput().
   Use setSlot() to set the level value for a particular DMX dimmer/address/channel.
   
   LX8266U0DMX is used with a single instance called ESP8266DMX0.
*/

class LX8266U0DMX {

  public:
  
	LX8266U0DMX ( void );
   ~LX8266U0DMX( void );
    

   /*!
    * @brief starts interrupt that continuously sends DMX output
    * @discussion Sets up baud rate, bits and parity, 
    *             sets globals accessed in ISR, 
    *             enables transmission and tx interrupt.
   */
   void startOutput( void );

   /*!
    * @brief starts interrupt that continuously sends DMX output both with UART0 and with LXESP8266DMX->UART1
    * @discussion Sets up baud rate, bits and parity, 
    *             sets globals accessed in ISR, 
    *             enables transmission and tx interrupt.  Also calls ESP8266DMX.startOutput();
   */
   void startDualOutput( void );
   
   
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
   
   /*!
    * @brief UART tx empty interrupt handler
   */
  	void		txEmptyInterruptHandler(void);
  	

    
  private:

	/*!
	 * @brief represents phase of sending dmx packet data/break/etc used to change baud settings
	 */
  	uint8_t  _dmx_send_state;
  	
	/*!
	 * @brief true when ISR is enabled
	 */
  	uint8_t  _interrupt_status;
  	
	/*!
	 * @brief count of idle interrupts
	 */
  	uint8_t  _idle_count;
  	
	/*!
	 * @brief pin used to control direction of output driver chip
	 */
  	uint8_t _direction_pin;
  	
	/*!
	 * @brief slot index indicating position of byte to be sent
	 */
  	uint16_t  _next_send_slot;
  	
	/*!
	 * @brief number of dmx slots ~24 to 512
	 */
  	uint16_t  _slots;
  	
	/*!
	 * @brief Array of dmx data including start code
	 */
  	uint8_t  _dmxData[DMX_MAX_FRAME];
  	
};

extern LX8266U0DMX ESP8266DMX0;

#endif // ifndef LX8266U0DMX_H
