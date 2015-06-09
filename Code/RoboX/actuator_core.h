 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * actuator_core.h
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-31
 * Edited:  2015-05-31
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

/// STRUCTS ///


/// GLOBALS ///
extern Servo SERVO_1;
extern Servo SERVO_2;

/// FUNCTIONS ///
void init_actuator_core(void);
void dc_drive(uint8_t motor_speed);
void dc_rotate(uint8_t motor_direction);
void servo_move(Servo to_rotate, uint8_t servo_position);

#endif
