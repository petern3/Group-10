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

////////////////
/// INCLUDES ///
////////////////
#include "Arduino.h"
#include "config.h"
#include <inttypes.h>
#include <Servo.h>
#include <Herkulex.h>

///////////////
/// DEFINES ///
///////////////
//#define STEPPER_1 1
//#define STEPPER_2 2
//#define STEPPER_3 3
//#define STEPPER_4 4

///////////////
/// CLASSES ///
///////////////
class DCMotor {
  private:
    Servo left_motor;
    Servo right_motor;
  public:
    uint8_t zero;
    void initialize(void);
    void calibrate(void);
    void drive(int8_t motor_speed, int8_t motor_rotation);  // absolute speed
};

class StepperMotor {
  private:
    int16_t steps_per_rev;
  public:
    uint8_t step_pin;
    uint8_t dir_pin;
    void initialize(uint8_t init_step_pin, uint8_t init_dir_pin, int16_t init_steps_per_rev);
    void rotate(int16_t num_degrees);  // relative position
};

class ServoMotor {
  private:
    Servo servo;
  public:
    void initialize(uint8_t init_port);
    void rotate(int16_t servo_position);  // absolute position
};

///////////////
/// GLOBALS ///
///////////////
extern DCMotor DC;
extern ServoMotor SERVO1;
extern ServoMotor SERVO2;
extern uint8_t SERVO_COLOUR;
extern StepperMotor STEPPER1;
extern StepperMotor STEPPER2;

/////////////////
/// FUNCTIONS ///
/////////////////
void init_actuator_core(void);
void extend_magnets(void);
void retract_magnets(void);
void toggle_magnets(void);

#endif
