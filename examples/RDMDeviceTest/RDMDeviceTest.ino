/**************************************************************************/
/*!
    @file     RDMDeviceTest.ino
    @author   Claude Heintz
    @license  BSD (see LXESP8266DMX LICENSE)
    @copyright 2017 by Claude Heintz

    Example showing LXESP8266DMX RDM support for devices.
    Control brightness of LED on GPIO14 with DMX address 1 (settable via RDM)
    
    @section  HISTORY
    v1.00 - First release  
*/
/**************************************************************************/
#include <LXESP8266UARTDMX.h>
#include <rdm/rdm_utility.h>
#include "ESP8266WiFi.h"

int got_dmx = 0;
int got_rdm = 0;
uint8_t discovery_enabled = 1;
uint16_t start_address = 1;

uint8_t device_label[33];

#define DEFAULT_DEVICE_LABEL  "RDM dev test v1.0"
#define MFG_LABEL             "LXDMX"
#define MODEL_DESCRIPTION     "RDMDeviceTest"

#define DIRECTION_PIN 15
#define LED_PIN       14

void setup() {
  WiFi.forceSleepBegin(); 
  delay(1);
  Serial.setDebugOutput(1); //use uart0 for debugging
  
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  //diagnostic pins
  pinMode(12, OUTPUT);
  pinMode(16, INPUT_PULLUP);

  strcpy((char*)device_label, DEFAULT_DEVICE_LABEL);
  
  ESP8266DMX.setDataReceivedCallback(&gotDMXCallback);
  ESP8266DMX.setRDMReceivedCallback(&gotRDMCallback);
  LX8266DMX::THIS_DEVICE_ID.setBytes(0x6C, 0x78, 0x0F, 0x0A, 0x0C, 0x0E);    //change device ID from default
  
  ESP8266DMX.startRDM(DIRECTION_PIN, DMX_TASK_RECEIVE);
}


// ***************** input callback function *************

ICACHE_RAM_ATTR void gotDMXCallback(int slots) {
  got_dmx = slots;
}

ICACHE_RAM_ATTR void gotRDMCallback(int len) {
  // rdm start code and checksum are validated before this is called
  got_rdm = len;
}

/************************************************************************

  The main loop checks to see if dmx input is available (got_dmx>0)
  And then reads the level of dimmer 1 to set PWM level of LED connected to pin 14
  
*************************************************************************/

void loop() {
  if ( got_dmx ) {
    
    //ESP8266 PWM is 10bit 0-1024
    analogWrite(LED_PIN,2*ESP8266DMX.getSlot(start_address));
    got_dmx = 0;  //reset
    
  } else if ( got_rdm ) {
    
    uint8_t* rdmdata = ESP8266DMX.receivedRDMData();
    
  uint8_t cmdclass = rdmdata[RDM_IDX_CMD_CLASS];
  uint16_t pid = (rdmdata[RDM_IDX_PID_MSB] << 8 ) | rdmdata[RDM_IDX_PID_LSB];
  

  if ( cmdclass == RDM_DISCOVERY_COMMAND ) {
    
    if ( pid == RDM_DISC_UNIQUE_BRANCH ) {
      if ( discovery_enabled ) {
        uint64_t tv = ESP8266DMX.THIS_DEVICE_ID.getValue();
        UID u;
        u.setBytes(&rdmdata[24]);                //lower 
        uint64_t uv = u.getValue();
        if ( tv >= uv ) {
          u.setBytes(&rdmdata[30]);              //upper
          uv = u.getValue();
          if ( tv <= uv ) {
            ESP8266DMX.sendRDMDiscoverBranchResponse();
          }
        }
      }
    } else {  // mute RDM_DISCOVERY_COMMAND PIDs
      UID destination;
      destination.setBytes(&rdmdata[RDM_IDX_DESTINATION_UID]);
      
      if ( pid == RDM_DISC_MUTE ) {
        if ( destination == ESP8266DMX.THIS_DEVICE_ID ) {
          discovery_enabled = 0;
          // send ACK
          UID source;
          source.setBytes(&rdmdata[RDM_IDX_SOURCE_UID]);
          ESP8266DMX.sendMuteAckRDMResponse(RDM_DISC_COMMAND_RESPONSE, source, RDM_DISC_MUTE);
        }
      } else if ( pid == RDM_DISC_UNMUTE ) {
        if ( destination == BROADCAST_ALL_DEVICES_ID ) {
          // just un-mute
          discovery_enabled = 1;
        } else if ( destination == ESP8266DMX.THIS_DEVICE_ID ) {
          discovery_enabled = 1;
          // send ACK
          UID source;
          source.setBytes(&rdmdata[RDM_IDX_SOURCE_UID]);
          ESP8266DMX.sendMuteAckRDMResponse(RDM_DISC_COMMAND_RESPONSE, source, RDM_DISC_UNMUTE);
        }
      }
      
    }
      
  } else if ( cmdclass == RDM_GET_COMMAND ) {
    UID destination;
    destination.setBytes(&rdmdata[RDM_IDX_DESTINATION_UID]);
          
    if ( destination == ESP8266DMX.THIS_DEVICE_ID ) {
       UID source;
       source.setBytes(&rdmdata[RDM_IDX_SOURCE_UID]);
       
       if ( pid == RDM_DEVICE_START_ADDR ) {
          
          uint8_t sa[2];
          sa[0] = start_address >> 8;
          sa[1] = start_address & 0xff;
          ESP8266DMX.sendRDMGetResponse(source, pid, sa, 2);
       } else if ( pid == RDM_DEVICE_MFG_LABEL ) {
          const char * label = MFG_LABEL;
          ESP8266DMX.sendRDMGetResponse(source, pid, (uint8_t*)label, 5);
       } else if ( pid == RDM_DEVICE_MODEL_DESC ) {
          const char * label = MODEL_DESCRIPTION;
          ESP8266DMX.sendRDMGetResponse(source, pid, (uint8_t*)label, 13);
       } else if ( pid == RDM_DEVICE_DEV_LABEL ) {
          ESP8266DMX.sendRDMGetResponse(source, pid, device_label, strlen((const char*)device_label));
       }

       
    }
  } else if ( cmdclass == RDM_SET_COMMAND ) {
      UID destination;
      destination.setBytes(&rdmdata[RDM_IDX_DESTINATION_UID]);
          
      if ( destination == ESP8266DMX.THIS_DEVICE_ID ) {
         UID source;
         source.setBytes(&rdmdata[RDM_IDX_SOURCE_UID]);
         
         if ( pid == RDM_DEVICE_START_ADDR ) {
            uint16_t scratch = (rdmdata[24] << 8) + rdmdata[25];
            if (( scratch > 0 ) && ( scratch < 513 )) {
              start_address = scratch;
            }
            ESP8266DMX.sendAckRDMResponse(RDM_SET_COMMAND_RESPONSE, source, pid);
            
         } else if ( pid == RDM_DEVICE_DEV_LABEL ) {
            uint8_t llen = 0;
            if ( rdmdata[2] > 24 ) {  //label not empty string
              llen = rdmdata[2] - 24;
              if ( llen > 32 ) {      //limit to max 32 characters
                llen = 32;
              }
            }
            for ( uint8_t j=0; j<33; j++) { //copy label, zero the rest of the array
              if ( j < llen ) {
                device_label[j] = rdmdata[24+j];
              } else {
                device_label[j] = 0;
              }
            }   // <-for
            ESP8266DMX.sendAckRDMResponse(RDM_SET_COMMAND_RESPONSE, source, pid);
         }      // <-pid RDM_DEVICE_DEV_LABEL
      }
  }
  got_rdm = 0;
  }               //gotRDM

  wdt_reset();
}
