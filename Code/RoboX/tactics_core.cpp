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
#include "interrupt_core.h"
#include "misc_core.h"
#include "sensor_core.h"


/// GLOBALS ///
uint8_t OPERATION_MODE = DEFAULT_MODE;

static int8_t FORWARD = 0;
static int8_t TURNING = 0;


/// FUNCTIONS ///
void init_tactics_core(void) {
  PRINT("\tTactics...");
  Timer3.initialize();  // in microseconds, 1 second by default
  PRINTLN("done");
}


void primary_tactic(void) {
  PRINTLN("Starting primary tactic:");
  Timer3.setPeriod(PRIMARY_TACTIC_PERIOD);
  Timer3.attachInterrupt(primary_tactic_ISR);
  
  while (!SD_ERROR.active && OPERATION_MODE == PRIMARY_MODE) {
    
    
  }
  PRINTLN("Primary tactic failure\n");
  Timer3.detachInterrupt();
}


void secondary_tactic(void) {
  PRINTLN("Starting secondary tactic:");
  Timer3.setPeriod(SECONDARY_TACTIC_PERIOD);
  Timer3.attachInterrupt(secondary_tactic_ISR);
  
  while(OPERATION_MODE == SECONDARY_MODE) {
    
    
    
    
  }
  PRINTLN("Secondary tactic failure\n");
  Timer3.detachInterrupt();
}


void manual_mode(void) {
  PRINTLN("Handing over control:");
  Timer3.setPeriod(MANUAL_CONTROL_PERIOD);
  Timer3.attachInterrupt(manual_control_ISR);
  
  char serial_byte = '\0';
  
  while(OPERATION_MODE == MANUAL_MODE) {
    #ifdef ENABLE_SERIAL
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
        PRINTLN("\tMoving Steppers forward by 180 degrees");
      }
      else if (serial_byte == STEPBWD) {
        stepper_rotate(STEPPER_1, -180);
        PRINTLN("\tMoving Steppers backward by 180 degrees");
      }
      
    }
    
    #endif
    
    dc_drive(FORWARD, TURNING);
    //PRINTLN(LEFT_ROTATION*0.104719755);
    //PRINTLN(RIGHT_ROTATION*0.104719755);
    //delay(200);
    
  }
  PRINTLN("Receiving control\n");
  Timer3.detachInterrupt();
}
