 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * tactics_core.cpp
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-06-17
 * Edited:  2015-06-18
 * 
 * Blurb
 * 
 *//////////////////////////////////////////////////////////////////////


/// INCLUDES ///
#include "tactics_core.h"
#include "actuator_core.h"
#include "exception_core.h"
//#include "misc_core.h"
#include "sensor_core.h"


/// GLOBALS ///
extern uint8_t OPERATION_MODE = DEFAULT_MODE;

static int8_t FORWARD = 0;
static int8_t TURNING = 0;


/// FUNCTIONS ///
void init_tactics_core(void) {
  PRINT("\tTactics...");
  Timer3.initialize(TIMERTHREE_PERIOD);  // in microseconds, 1 second by default
  PRINTLN("done");
}


void primary_tactic(void) {
  PRINTLN("Starting primary tactic:");
  Timer3.detachInterrupt();
  
  while (!SD_ERROR.active && OPERATION_MODE == PRIMARY_MODE) {
    
    read_sensors();
    
    
  }
  PRINTLN("Primary tactic failure\n");
  Timer3.attachInterrupt(read_sensors, SENSOR_READ_PERIOD);
}


void secondary_tactic(void) {
  PRINTLN("Starting secondary tactic:");
  
  while(OPERATION_MODE == SECONDARY_MODE) {
    
  }
  PRINTLN("Secondary tactic failure\n");
  
}


void manual_mode(void) {
  PRINTLN("Handing over control:");
  char serial_byte = '\0';
  
  while(OPERATION_MODE == MANUAL_MODE) {
    if (Serial.available() > 0) {
      serial_byte = Serial.read();
      
      if (serial_byte == FWD) {
        FORWARD += FORWARD_INCREMENT;
        if (FORWARD > 90) {
          FORWARD = 90;
        }
        PRINT("\tForward speed value is ");
        PRINTLN(FORWARD);
      }
      else if (serial_byte == BWD) {
        FORWARD -= FORWARD_INCREMENT;
        if (FORWARD < -90) {
          FORWARD = -90;
        }
        PRINT("\tForward speed value is ");
        PRINTLN(FORWARD);
      }
      else if (serial_byte == LFT) {
        TURNING += TURNING_INCREMENT;
        if (TURNING > 90) {
          TURNING = 90;
        }
        PRINT("\tTurning speed value is ");
        PRINTLN(TURNING);
      }
      else if (serial_byte == RHT) {
        TURNING -= TURNING_INCREMENT;
        if (TURNING < -90) {
          TURNING = -90;
        }
        PRINT("\tTurning speed value is ");
        PRINTLN(TURNING);
      }
      else if (serial_byte == STP) {
        FORWARD = 0;
        TURNING = 0;
        PRINTLN("\tStopping");
      }
      else if (serial_byte == STEPFWD) {
        stepper_rotate(STEPPER_1, 180);
        PRINTLN("\tMoving Stepper 1 forward by 180 degrees");
      }
      else if (serial_byte == STEPBWD) {
        stepper_rotate(STEPPER_1, -180);
        PRINTLN("\tMoving Stepper 1 backward by 180 degrees");
      }
    }
    
    dc_drive(FORWARD, TURNING);
    
  }
  PRINTLN("Receiving control\n");
}
