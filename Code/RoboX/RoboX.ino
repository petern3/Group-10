 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * RoboX.ino
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-26
 * Edited:  2015-06-18
 * 
 * This file is the main file, contatining the overall initialization
 * sequence and program kernel.
 * 
 *//////////////////////////////////////////////////////////////////////


/// DEFINES ///
#include "Arduino.h"
#include <inttypes.h>
#include <Servo.h>
#include <Herkulex.h>
#include <Stepper.h>
#include <SPI.h>
#include <SD.h>
#include <TimerThree.h>

#include "config.h"
#include "actuator_core.h"
#include "exception_core.h"
#include "map_core.h"
#include "misc_core.h"
#include "sensor_core.h"
#include "tactics_core.h"
#include "voice_core.h"


/// INITIALIZATION ///
void setup() {
  Serial.begin(BAUD_RATE);
  PRINTLN("\nInitializing:");
  
  init_actuator_core();
  init_exception_core();
  init_map_core();
  init_sensor_core();
  init_tactics_core();
  init_voice_core();
  
  PRINTLN("Ready!\n");
  
  display_map();
}


/// KERNEL ///
void loop() {
  switch (OPERATION_MODE) {
    case PRIMARY_MODE:
      primary_tactic();
      break;
    case MANUAL_MODE:
      manual_mode();
      break;
    default:
      secondary_tactic();
  }
}

