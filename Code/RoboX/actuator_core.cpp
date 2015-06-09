
 


#include "actuator_core.h"


/// GLOBALS ///
static Servo LEFT_DRIVE;
static Servo RIGHT_DRIVE;

Servo SERVO_1;
Servo SERVO_2;


/// FUNCTIONS ///
void init_actuator_core(void) {
  // DC Motors
  LEFT_DRIVE.attach(DC_LEFT_PIN);
  RIGHT_DRIVE.attach(DC_RIGHT_PIN);
  
  // Servos
  SERVO_1.attach(SERVO1_PIN);
  SERVO_2.attach(SERVO2_PIN);
  
  // Stepper Motors
  pinMode(STEPPER1_STEP_PIN,OUTPUT);
  pinMode(STEPPER1_DIR_PIN,OUTPUT);
  pinMode(STEPPER2_STEP_PIN,OUTPUT);
  pinMode(STEPPER2_DIR_PIN,OUTPUT);
  pinMode(STEPPER3_STEP_PIN,OUTPUT);
  pinMode(STEPPER3_DIR_PIN,OUTPUT);
  pinMode(STEPPER4_STEP_PIN,OUTPUT);
  pinMode(STEPPER4_DIR_PIN,OUTPUT);
  
}


void dc_drive(uint8_t motor_speed) {
  
  LEFT_DRIVE.write(motor_speed);
  RIGHT_DRIVE.write(motor_speed);
}


void dc_rotate(uint8_t motor_direction) {
  
  if (motor_direction == DC_LEFT_FAST) {
    LEFT_DRIVE.write(DC_BWD_FAST);
    RIGHT_DRIVE.write(DC_FWD_FAST);
  } else if (motor_direction == DC_LEFT_SLOW) {
    LEFT_DRIVE.write(DC_BWD_SLOW);
    RIGHT_DRIVE.write(DC_FWD_SLOW);
  } else if (motor_direction == DC_ZERO) {
    LEFT_DRIVE.write(DC_ZERO);
    RIGHT_DRIVE.write(DC_ZERO);
  } else if (motor_direction == DC_RIGHT_SLOW) {
    LEFT_DRIVE.write(DC_FWD_SLOW);
    RIGHT_DRIVE.write(DC_BWD_SLOW);
  } else if (motor_direction == DC_RIGHT_FAST) {
    LEFT_DRIVE.write(DC_FWD_FAST);
    RIGHT_DRIVE.write(DC_BWD_FAST);
  }
}

void servo_move(Servo to_rotate, uint8_t servo_position) {
  if (servo_position < 180) {
    to_rotate.write(servo_position);
  } else {
    to_rotate.write(180);
  }
}


