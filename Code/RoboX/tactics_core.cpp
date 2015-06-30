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


/// GLOBALS ///
extern bool FORCE_SECONDARY = false;


/// FUNCTIONS ///
void init_tactics_core(void) {
  
}


void primary_tactic(void) {
  PRINTLN("Starting primary tactic:");
  
  while (!SD_ERROR.active && !FORCE_SECONDARY) {
    read_sensors();
  }
  PRINTLN("Primary tactic failure\n");
}

void secondary_tactic(void) {
  PRINTLN("Starting secondary tactic:");
  
  while(true) {
    read_sensors();
    //PRINTLN(IR_SHT1.sensor_value);
    //PRINTLN(IR_SHT2.sensor_value);
    //PRINTLN();
    
    //delay(1000);
    
  }
  PRINTLN("Secondary tactic failure\n");
}


