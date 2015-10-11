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

//////////////
/// PINOUT ///
//////////////
// analogWrite() will not work on pins 9 and 10 due to TimerThree
// analogWrite() may not work on pins 11 and 3 due to Speaker
#define DC_LEFT_PIN 12
#define DC_RIGHT_PIN 13
#define DC_LEFT_INTERRUPT_PIN 5  // to check
#define DC_RIGHT_INTERRUPT_PIN 4

#define SERVO1_PIN 3
#define SERVO2_PIN 2

#define SMART_SERVO1_ADDRESS 0xFD

#define STEPPER1_STEP_PIN 42
#define STEPPER1_DIR_PIN 43
#define STEPPER2_STEP_PIN 44
#define STEPPER2_DIR_PIN 45

#define IR_SHT1_PIN A8 //10
//#define IR_SHT2_PIN A1 //11
#define IR_MED1_PIN A2
#define IR_MED2_PIN A3
#define IR_LNG1_PIN A4
#define IR_LNG2_PIN A0
#define USONIC3_TRIG_PIN 7
#define USONIC3_ECHO_PIN 6
#define USONIC2_TRIG_PIN 11
#define USONIC2_ECHO_PIN 10
#define USONIC1_TRIG_PIN 9
#define USONIC1_ECHO_PIN 8
#define SONAR1_PIN A6

#define IR_VAR1_PIN 26 // 30  // left
#define IR_VAR2_PIN 28 // 32  // middle
#define IR_VAR3_PIN 30 // 34  // right
#define LIMIT_O_PIN 32 // 36
#define TURN_ON_PIN A9

#define CHIP_SELECT_PIN 53
#define SPEAKER_PIN 6

#define DIP8_S1_PIN 38 // WORKS - PRIMARY/MANUAL
#define DIP8_S2_PIN 39 // DOESNT
#define DIP8_S3_PIN 40 // WORKS - MOTOR ENABLE
#define DIP8_S4_PIN 41 // WORKS
#define DIP8_S5_PIN 34 // WORKS
#define DIP8_S6_PIN 35 // DOESNT
#define DIP8_S7_PIN 36 // WORKS
#define DIP8_S8_PIN 37 // DOESNT


////////////////////////
/// SCHEDULER CONFIG ///
////////////////////////
//#define TIMERTHREE_PERIOD 1000  // microseconds
#define PRIMARY_TACTIC_PERIOD 1000000  // microseconds  // 1Hz
#define SECONDARY_TACTIC_PERIOD 50000  // microseconds  // 20Hz
#define MANUAL_CONTROL_PERIOD 100000  // microseconds   // 10Hz

#define IDLE_TIMEOUT 10000  // milliseconds

///////////////////////////
/// USER CONTROL CONFIG ///
///////////////////////////
#define FORWARD_INCREMENT 10
#define TURNING_INCREMENT 10

#define FWD 'w'
#define BWD 's'
#define LFT 'a'
#define RHT 'd'
#define STP 'x'
#define EXTEND 'h'
#define RETRACT 'g'
#define TOGGLE 't'
#define RAISE 'r'
#define LOWER 'f'

///////////////////////
/// ACTUATOR CONFIG ///
///////////////////////
/// DC MOTOR CONFIG ///
#define ENCODER_PPR 330  // 663
#define ENCODER_RESET_TIMEOUT 10000 // microseconds
#define DC_FORWARD true
#define DC_BACKWARD false
#define AUTO_CALIBRATE_DC
#define DC_CALIBRATION_LIMIT 15 // 90 +/- n

#define SPEED_P 0.2
#define ROTATE_P 0.35
#define ANGLE_LIMIT 45

/// STEPPER MOTOR CONFIG ///
#define STEPPER1_SPR 1036 // 630/1.8*5.18 Geared
#define STEPPER2_SPR 1036 // Geared
//#define STEPPER3_SPR 200  // to calibrate
//#define STEPPER4_SPR 200  // to calibrate

/// SERVO CONFIG ///
#define MAX_TRAVEL 160

/*
  LED_GREEN
  LED_BLUE
  LED_CYAN
  LED_RED
  LED_GREEN2
  LED_PINK
  LED_WHITE
*/

/////////////////////
/// SENSOR CONFIG ///
/////////////////////
// USONIC 1 is left
// USONIC 2 is right
// USONIC 3 is upper centre
// IR_MED1 left one
// IR_MED2 right one
#define IR_SHT1_OFFSET {-80, -80}
#define IR_SHT1_ANGLE -90
#define IR_MED1_OFFSET {-100, 100}
#define IR_MED1_ANGLE 35
#define IR_MED2_OFFSET {100, 100}
#define IR_MED2_ANGLE -35
#define IR_LNG1_OFFSET {0, 60}
#define IR_LNG1_ANGLE 0
#define IR_LNG2_OFFSET {0, 0}
#define IR_LNG2_ANGLE 0
#define USONIC1_OFFSET {135, -150}
#define USONIC1_ANGLE 50
#define USONIC2_OFFSET {-135, -150}
#define USONIC2_ANGLE -50
#define USONIC3_OFFSET {0, 50}
#define USONIC3_ANGLE 0
#define SONAR1_OFFSET {0, 50}
#define SONAR1_ANGLE 0


#define IR_SHT_MIN_ADC 0   // To measure
#define IR_SHT_DV1_ADC 200   // to measure
#define IR_SHT_DV2_ADC 400   // to measure
#define IR_SHT_MAX_ADC 1023  // To measure
#define IR_SHT_MIN_MM 40
#define IR_SHT_DV1_MM 120
#define IR_SHT_DV2_MM 200
#define IR_SHT_MAX_MM 300

#define IR_MED_MIN_ADC 976   // To measure
#define IR_MED_DV1_ADC 380   // to measure
#define IR_MED_DV2_ADC 237   // to measure
#define IR_MED_MAX_ADC 164  // To measure
#define IR_MED_MIN_MM 100
#define IR_MED_DV1_MM 300
#define IR_MED_DV2_MM 500
#define IR_MED_MAX_MM 800

#define IR_LNG_MIN_ADC 930   // To measure
#define IR_LNG_DV1_ADC 575   // to measure
#define IR_LNG_DV2_ADC 375   // to measure 
#define IR_LNG_MAX_ADC 240  // To measure
#define IR_LNG_MIN_MM 200
#define IR_LNG_DV1_MM 400
#define IR_LNG_DV2_MM 600
#define IR_LNG_MAX_MM 1000

#define SONAR_MIN_ADC 132   // To measure
#define SONAR_DV1_ADC 153   // to measure
#define SONAR_DV2_ADC 170   // to measure
#define SONAR_MAX_ADC 200  // To measure
#define SONAR_MIN_MM 300
#define SONAR_DV1_MM 375
#define SONAR_DV2_MM 425
#define SONAR_MAX_MM 500

#define USONIC_TIMEOUT 4060ul //5.8*700mm, microseconds
#define SENSOR_BUFFER_SIZE 10
#define TARGET_TIMEOUT_BUFFER_SIZE 20

//////////////////////
/// TACTICS CONFIG ///
//////////////////////

#define BACKING_ANGLE 179 //120  // degrees, set to 181 to disable

#define WEIGHT_DETECT_TOLERANCE 120 // mm
#define CENTRE_SENSOR_TOLERANCE 200 // mm

#define GREEN_THRESHOLD 180
#define BLUE_THRESHOLD 150

#define WEIGHT_TIMEOUT_MAX 4000  // milliseconds
#define WEIGHT_TIMEOUT_INC 400   // milliseconds

#define STOP_BUFFER_SIZE 60

//////////////////
/// MAP CONFIG ///
//////////////////
#define MAP_SIZE_X 5000 //4900  // mm  Wider than is long
#define MAP_SIZE_Y 2500 //2400  // mm
#define MAP_SPACING 250 //20  // mm

#define ROBOT_RADIUS 270  // mm
#define ROBOT_DIAMETER 540  // mm

////////////////////
/// DEBUG CONFIG ///
////////////////////
#define BAUD_RATE 115200
#define ENABLE_SERIAL // define if you want to send serial

#ifdef ENABLE_SERIAL
  #define PRINT(s) Serial.print(s)
  #define PRINTLN(s) Serial.println(s)
#else
  #define PRINT(s)
  #define PRINTLN(s)
#endif

#define ENABLE_SOUNDS  // define if you want to play sounds
//#define DEBUG_ERROR_SOUNDS  // define if you only want error sounds

//#define DEBUG_MAP_NUMBERS  // define if you want to display numbers instead of symbols
#ifndef DEBUG_MAP_NUMBERS
  //#define NODE_EDGE "###"
  #define NODE_OPEN "   "
  //#define NODE_WALL "[ ]"
  #define NODE_BASE "~~~"
  #define NODE_PACK ":::"
#endif

////////////////////
/// ERROR CONFIG ///
////////////////////
#define NULL_ERROR_SOUND "\0"


#endif
