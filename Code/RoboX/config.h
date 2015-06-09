 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * config.h
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-26
 * Edited:  2015-05-31
 * 
 * Blurb
 * 
 *//////////////////////////////////////////////////////////////////////
 

#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"

/// PINOUT ///
#define DC_LEFT_PIN 12
#define DC_RIGHT_PIN 13

#define STEPPER1_STEP_PIN 30
#define STEPPER1_DIR_PIN 31
#define STEPPER2_STEP_PIN 32
#define STEPPER2_DIR_PIN 33
#define STEPPER3_STEP_PIN 34
#define STEPPER3_DIR_PIN 35
#define STEPPER4_STEP_PIN 36
#define STEPPER4_DIR_PIN 37

#define SERVO1_PIN 3
#define SERVO2_PIN 2

#define IR_MED1_PIN A10

/// SCHEDULER CONFIG ///



/// DC MOTOR CONFIG ///
#define DC_FWD_FAST 110
#define DC_FWD_SLOW 100
#define DC_ZERO 90
#define DC_BWD_SLOW 80
#define DC_BWD_FAST 70

#define DC_LEFT_FAST 0
#define DC_LEFT_SLOW 1
#define DC_RIGHT_SLOW 2
#define DC_RIGHT_FAST 3


/// MAP CONFIG ///
#define MAP_SIZE_X 5000  //4900  // mm  Wider than is long
#define MAP_SIZE_Y 2500  //2400  // mm
#define MAP_SPACING 500  //20  // mm


/// DEBUG CONFIG ///
#define BAUD_RATE 9600
//#define ENABLE_SOUNDS  // define if you want to play sounds
//#define DEBUG_ERROR_SOUNDS  // define if you only want error sounds

//#define DEBUG_MAP_NUMBERS  // define if you want to diplay numbers instead of symbols
#ifndef DEBUG_MAP_NUMBERS
  #define NODE_EDGE "##"
  #define NODE_OPEN "  "
  #define NODE_WALL "[]"
  #define NODE_BASE "~~"
  #define NODE_PACK "::"
#endif


/// ERROR CONFIG ///
#define NULL_ERROR_SOUND "\0"

/// SENSOR CONFIG ///
#define IR_VAR_RANGE 0
#define IR_SHT_RANGE 1
#define IR_MED_RANGE 2
#define IR_LNG_RANGE 3
#define ULTRASONIC 4
#define SONAR 5

#define IR_VAR_MIN 0
#define IR_VAR_MAX 1000

#define IR_SHT_MIN 40
#define IR_SHT_MAX 300

#define IR_MED_MIN 100
#define IR_MED_MAX 800

#define IR_LNG_MIN 50
#define IR_LNG_MAX 2000


#endif
