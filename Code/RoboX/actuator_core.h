 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * actuator_core.h
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-31
 * Edited:  2015-06-18
 * 
 * The header file of the Actuator Core.
 * 
 *//////////////////////////////////////////////////////////////////////
 

#ifndef ACTUATOR_CORE_H
#define ACTUATOR_CORE_H


/// INCLUDES ///
#include "Arduino.h"
#include "config.h"
#include <inttypes.h>
#include <Servo.h>
#include <Herkulex.h>

/// DEFINES ///
#define STEPPER1 1
#define STEPPER2 2
#define STEPPER3 3
#define STEPPER4 4


/// STRUCTS ///


/// GLOBALS ///
extern float TAU;

extern Servo SERVO_1;
extern Servo SERVO_2;

/// FUNCTIONS ///
void init_actuator_core(void);
void dc_drive(int8_t motor_speed, int8_t motor_rotation);
//void dc_drive(uint8_t motor_speed);       // Obselete
//void dc_rotate(uint8_t motor_direction);  // Obselete
void servo_rotate(Servo to_rotate, uint8_t servo_position);
void stepper_rotate(uint8_t to_rotate, int16_t num_degrees);

#endif
