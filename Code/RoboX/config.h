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


#define IR_SHT1_PIN A10
#define IR_SHT2_PIN A11
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
#define SENSOR_PERIOD 10  // milliseconds
#define THINKING_PERIOD 100  // milliseconds


/// DC MOTOR CONFIG ///
#define ENCODER_PPR 663
#define ENCODER_WRAP 10  // Number of revolutions before wrapping
#define DC_FORWARD true


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
#define MAP_SIZE_X 5000  //4900  // mm  Wider than is long
#define MAP_SIZE_Y 2500  //2400  // mm
#define MAP_SPACING 500  //20  // mm

#define DIR_BUFFER 20
#define MAP_DIR "BOTMAP/"  // Format is "BOTMAP/xxx,yyy" with no file extention
#define EMPTY 0x00          // Soft or hard
#define WEIGHT 0x01         // |
#define SOFT_WALL_MIN 0x04  // 00000100
#define SOFT_WALL_MAX 0x7F  // 01111111
#define HARD_WALL_MIN 0x84  // 10000100
#define HARD_WALL_MAX 0xFF  // 11111111
#define WALL_INCR 1


/// DEBUG CONFIG ///
#define BAUD_RATE 115200
#define ENABLE_SERIAL  // define if you want to send serial
//#define ENABLE_SOUNDS  // define if you want to play sounds
//#define DEBUG_ERROR_SOUNDS  // define if you only want error sounds

//#define DEBUG_MAP_NUMBERS  // define if you want to display numbers instead of symbols
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
#define IR_SHT_MIN_MM 40
#define IR_SHT_MAX_MM 300
#define IR_SHT_MIN_ADC 0     // To measure
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
