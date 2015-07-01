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
#include "misc_core.h"
#include "sensor_core.h"
#include <TimerThree.h>


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
  
  while(OPERATION_MODE == MANUAL_MODE) {
    if (Serial.read() == 'w') {
      FORWARD += FORWARD_INCREMENT;
    }
    if (Serial.read() == 's') {
      FORWARD -= FORWARD_INCREMENT;
    }
    if (Serial.read() == 'a') {
      TURNING += TURNING_INCREMENT;
    }
    if (Serial.read() == 'd') {
      TURNING -= TURNING_INCREMENT;
    }
    if (Serial.read() == 'x') {
      FORWARD = 0;
      TURNING = 0;
    }
    if (Serial.read() == 'r') {
      stepper_rotate(STEPPER_1, 180);
    }
    if (Serial.read() == 'f') {
      stepper_rotate(STEPPER_1, -180);
    }
    
    dc_drive(FORWARD, TURNING);
    
  }
  PRINTLN("Receiving control\n");
}
