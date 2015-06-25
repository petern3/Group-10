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
//#include "exception_core.h"
#include "sensor_core.h"


/// GLOBALS ///
extern bool FORCE_SECONDARY = false;


/// FUNCTIONS ///
void init_tactics_core(void) {
  
}


void primary_tactic(void) {
  while (!SD_ERROR.active && !FORCE_SECONDARY) {
    if (millis() > SENSOR_PERIOD) {
      read_sensors();
    }
    if (millis() > THINKING_PERIOD) {
      
    }
  }
}

void secondary_tactic(void) {
  if (millis() > SENSOR_PERIOD) {
    read_sensors();
  }
  if (millis() > THINKING_PERIOD) {
    
  }
}


