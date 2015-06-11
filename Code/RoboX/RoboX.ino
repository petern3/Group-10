 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * RoboX.ino
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-26
 * Edited:  2015-05-31
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

#include "config.h"
#include "actuator_core.h"
#include "exception_core.h"
#include "map_core.h"
#include "sensor_core.h"
#include "voice_core.h"


/// INITIALIZATION ///
void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(BAUD_RATE);
  init_actuator_core();
  init_exception_core();
  init_map_core();
  init_sensor_core();
  init_voice_core();
  
  //display_map();
  
}


/// KERNEL ///
void loop() {
  // put your main code here, to run repeatedly:
  
  Serial.println();
  
  delay(1000);
  
  delay(1000);
  
}

