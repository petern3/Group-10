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
#include <SPI.h>
#include <SD.h>

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
  
  //sd_test1();
  //sd_test2();
  //display_map();
  
}


/// KERNEL ///
void loop() {
  // put your main code here, to run repeatedly:
  //report_exception(&SD_ERROR);
  //Serial.println(sensor_distance(IR_MED1));
  //Serial.println(analogRead(IR_MED1_PIN));
  delay(10);
  //Serial.println(sensor_distance(IR_LNG1));
  //Serial.println(analogRead(IR_LNG1_PIN));
  //Serial.println();
  
  delay(10);
  
  
}

