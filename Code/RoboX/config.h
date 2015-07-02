 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * config.h
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-26
 * Edited:  2015-06-18
 * 
 * Blurb
 * 
 *//////////////////////////////////////////////////////////////////////
 

#ifndef CONFIG_H
#define CONFIG_H

#include "Arduino.h"

/// PINOUT ///
//Analog write will not work on pins 9 and 10 due to TimerThree!
#define DC_LEFT_PIN 12
#define DC_RIGHT_PIN 13
#define DC_LEFT_INTERRUPT 4
#define DC_RIGHT_INTERRUPT 5

#define SERVO1_PIN 3
#define SERVO2_PIN 2

#define SMART_SERVO1_ADDRESS 0xFE

#define STEPPER1_STEP_PIN 30
#define STEPPER1_DIR_PIN 31
#define STEPPER2_STEP_PIN 32
#define STEPPER2_DIR_PIN 33
#define STEPPER3_STEP_PIN 34
#define STEPPER3_DIR_PIN 35
#define STEPPER4_STEP_PIN 36
#define STEPPER4_DIR_PIN 37


#define IR_SHT1_PIN A0 //10
#define IR_SHT2_PIN A1 //11
#define IR_MED1_PIN A9
#define IR_MED2_PIN A8
#define IR_LNG1_PIN A7
#define IR_LNG2_PIN A6
#define USONIC1_PIN A5
#define USONIC2_PIN A4
#define SONAR_PIN A3

#define IR_VAR1_PIN 29
#define IR_VAR2_PIN 28
#define IR_VAR3_PIN 27


#define CHIP_SELECT_PIN 53


/// SCHEDULER CONFIG ///
#define PRIMARY_MODE 0
#define SECONDARY_MODE 1
#define MANUAL_MODE 2
#define DEFAULT_MODE MANUAL_MODE  // can change to any of the previous modes

#define TIMERTHREE_PERIOD 1000  // microseconds
#define SENSOR_READ_PERIOD 500000  // microseconds. Not used in primary tactic.


/// DC MOTOR CONFIG ///
#define ENCODER_PPR 663
#define ENCODER_WRAP 1  // Number of revolutions before wrapping
#define DC_FORWARD true
#define DC_BACKWARD false


/// USER CONTROL CONFIG ///
#define FORWARD_INCREMENT 10
#define TURNING_INCREMENT 10

#define FWD 'w'
#define BWD 's'
#define LFT 'a'
#define RHT 'd'
#define STP 'x'
#define STEPFWD 'r'
#define STEPBWD 'f'


/// STEPPER MOTOR CONFIG ///
#define STEPPER1_SPR 200
//#define STEPPER1_INV
#define STEPPER2_SPR 200
//#define STEPPER2_INV
#define STEPPER3_SPR 1040  // Geared
//#define STEPPER3_INV
#define STEPPER4_SPR 1000  // Geared
//#define STEPPER4_INV

/// MAP CONFIG ///
#define MAP_SIZE_X 4900 //4900  // mm  Wider than is long
#define MAP_SIZE_Y 2400 //2400  // mm
#define MAP_SPACING 200 //20  // mm

#define ROBOT_RADIUS 240  // mm
#define ROBOT_DIAMETER 480  // mm


#define MAP_DIR_BASE "BOTMAP"  // Format is "BOTMAPn/x.y" with no file extention
#define DIR_BUFFER 22

#define EMPTY 0x00
#define PACK  0x01     // Soft or hard
#define BASE  0x02     //      |
#define WALL_MIN 0x04  //      00000100
#define WALL_MAX 0x7F  //      01111111
#define WALL_INCR 1

#define HARDNESS_BIT 0x80
#define SOFT_WALL 0
#define HARD_WALL 1


/// DEBUG CONFIG ///
#define BAUD_RATE 115200
#define ENABLE_SERIAL // define if you want to send serial

#ifdef ENABLE_SERIAL
  #define PRINT(s) Serial.print(s)
  #define PRINTLN(s) Serial.println(s)
#else
  #define PRINT(s)
  #define PRINTLN(s)
#endif


//#define ENABLE_SOUNDS  // define if you want to play sounds
//#define DEBUG_ERROR_SOUNDS  // define if you only want error sounds

//#define DEBUG_MAP_NUMBERS  // define if you want to display numbers instead of symbols
#ifndef DEBUG_MAP_NUMBERS
  //#define NODE_EDGE "###"
  #define NODE_OPEN "   "
  //#define NODE_WALL "[ ]"
  #define NODE_BASE "~~~"
  #define NODE_PACK ":::"
#endif


/// ERROR CONFIG ///
#define NULL_ERROR_SOUND "\0"


/// SENSOR CONFIG ///
#define IR_SHT_MIN_MM 40
#define IR_SHT_MAX_MM 300
#define IR_SHT_MIN_ADC 157     // To measure
#define IR_SHT_MAX_ADC 1023  // To measure

#define IR_MED_MIN_MM 100
#define IR_MED_MAX_MM 800
#define IR_MED_MIN_ADC 0     // To measure
#define IR_MED_MAX_ADC 1023  // To measure

#define IR_LNG_MIN_MM 50     // To find
#define IR_LNG_MAX_MM 2000   // To find
#define IR_LNG_MIN_ADC 0     // To measure
#define IR_LNG_MAX_ADC 1023  // To measure


#endif
