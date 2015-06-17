 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * tactics_core.cpp
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-06-17
 * Edited:  2015-06-17
 * 
 * Blurb
 * 
 *//////////////////////////////////////////////////////////////////////


/// INCLUDES ///
#include "tactics_core.h"
#include "sensor_core.h"


/// GLOBALS ///


/// FUNCTIONS ///
void init_tactics_core(void) {
  
}


void primary_tactic(void) {
  if (millis() > SENSOR_PERIOD) {
    read_sensors();
  }
  if (millis() > THINKING_PERIOD) {
    
    
    
    
  }
}




