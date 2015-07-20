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
// analogWrite() will not work on pins 9 and 10 due to TimerThree
// analogWrite() may not work on pins 11 and 3 due to Speaker
#define DC_LEFT_PIN 12
#define DC_RIGHT_PIN 13
#define DC_LEFT_INTERRUPT_PIN 4
#define DC_RIGHT_INTERRUPT_PIN 5

#define SERVO1_PIN 3
#define SERVO2_PIN 2

#define SMART_SERVO1_ADDRESS 0xFE

#define STEPPER1_STEP_PIN 42
#define STEPPER1_DIR_PIN 43
#define STEPPER2_STEP_PIN 44
#define STEPPER2_DIR_PIN 45
//#define STEPPER3_STEP_PIN 42
//#define STEPPER3_DIR_PIN 43
//#define STEPPER4_STEP_PIN 44
//#define STEPPER4_DIR_PIN 45


#define IR_SHT1_PIN A0 //10
#define IR_SHT2_PIN A1 //11
#define IR_MED1_PIN A9
#define IR_MED2_PIN A8
#define IR_LNG1_PIN A7
#define IR_LNG2_PIN A6
#define USONIC1_TRIG_PIN 11
#define USONIC1_ECHO_PIN 10
#define USONIC2_TRIG_PIN 11
#define USONIC2_ECHO_PIN 10
#define SONAR_PIN A3

#define IR_VAR1_PIN 29
#define IR_VAR2_PIN 28
#define IR_VAR3_PIN 27

#define CHIP_SELECT_PIN 53

#define SPEAKER_PIN 8


/// SCHEDULER CONFIG ///
#define PRIMARY_MODE 0
#define SECONDARY_MODE 1
#define MANUAL_MODE 2
#define DEFAULT_MODE MANUAL_MODE  // can change to any of the previous modes

//#define TIMERTHREE_PERIOD 1000  // microseconds
#define PRIMARY_TACTIC_PERIOD 1000000  // microseconds
#define SECONDARY_TACTIC_PERIOD 100000  // microseconds
#define MANUAL_CONTROL_PERIOD 1000000  // microseconds


/// DC MOTOR CONFIG ///
#define ENCODER_PPR 330  // 663
#define ENCODER_RESET_TIMEOUT 10000 // microseconds
#define DC_FORWARD true
#define DC_BACKWARD false
#define DC_CALIBRATION_LIMIT 20 // 90 +/- 20


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
#define STEPPER1_SPR 1040 // Geared
#define STEPPER1_INV
#define STEPPER2_SPR 1040 // Geared
//#define STEPPER2_INV
#define STEPPER3_SPR 200  // to calibrate
//#define STEPPER3_INV
#define STEPPER4_SPR 200  // to calibrate
//#define STEPPER4_INV

/// MAP CONFIG ///
#define MAP_SIZE_X 4900 //4900  // mm  Wider than is long
#define MAP_SIZE_Y 2400 //2400  // mm
#define MAP_SPACING 200 //20  // mm

#define ROBOT_RADIUS 240  // mm
#define ROBOT_DIAMETER 480  // mm


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
#define IR_SHT_MIN_ADC 157    // To measure
#define IR_SHT_MAX_ADC 1023  // To measure

#define IR_MED_MIN_MM 100
#define IR_MED_MAX_MM 800
#define IR_MED_MIN_ADC 0     // To measure
#define IR_MED_MAX_ADC 1023  // To measure

#define IR_LNG_MIN_MM 200
#define IR_LNG_MAX_MM 1500
#define IR_LNG_MIN_ADC 0     // To measure
#define IR_LNG_MAX_ADC 1023  // To measure

#define USONIC_MIN_MM 20
#define USONIC_MAX_MM 4000
#define USONIC_MIN_ADC 0     // To measure
#define USONIC_MAX_ADC 1023  // To measure


#endif
