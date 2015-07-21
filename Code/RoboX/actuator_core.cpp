 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * actuator_core.cpp
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-31
 * Edited:  2015-06-18
 * 
 * Blurb
 * 
 *//////////////////////////////////////////////////////////////////////


#include "actuator_core.h"
#include "exception_core.h"
#include "interrupt_core.h"
#include "misc_core.h"

///////////////
/// GLOBALS ///
///////////////
DCMotor DC;

ServoMotor SERVO1;
ServoMotor SERVO2;

StepperMotor STEPPER1;
StepperMotor STEPPER2;

bool is_extended = true;

/////////////////
/// FUNCTIONS ///
/////////////////
void init_actuator_core(void) {
  PRINT("\tDC Motors...");
  
  // DC Motors
  DC.initialize();
  
  attachInterrupt(DC_LEFT_INTERRUPT_PIN, left_encoder_ISR, RISING);  // can do 'CHANGE', but would include twice the interrupts (high CPU load)
  attachInterrupt(DC_RIGHT_INTERRUPT_PIN, right_encoder_ISR, RISING);
  
  DC.calibrate();
  
  if (DC_MOTOR_ERROR.is_active()) {
    PRINTLN("failed");
  } else {
    PRINT("zero set to ");
    PRINTLN(DC.zero);
  }
  
  PRINT("\tActuators...");
  // Servos
  SERVO1.initialize(SERVO1_PIN);
  SERVO2.initialize(SERVO2_PIN);
  
  // Stepper Motors
  STEPPER1.initialize(STEPPER1_STEP_PIN, STEPPER1_DIR_PIN, STEPPER1_SPR);
  STEPPER2.initialize(STEPPER2_STEP_PIN, STEPPER2_DIR_PIN, STEPPER2_SPR);
  
  // Smart Servos
  Herkulex.beginSerial2(115200);  // When in port C2 for Transmit/Receive #2
  Herkulex.reboot(SMART_SERVO1_ADDRESS);
  Herkulex.initialize();
  
  PRINTLN("done");
}

////////////////////////////////
/// DC MOTOR CLASS FUNCTIONS ///
////////////////////////////////
void DCMotor::initialize(void) {
  this->zero = 90;
  this->left_motor.attach(DC_LEFT_PIN);
  this->right_motor.attach(DC_RIGHT_PIN);
}

void DCMotor::calibrate(void) {
  this->left_motor.write(this->zero);
  this->right_motor.write(this->zero);
  uint8_t dc_high_limit = this->zero;
  uint8_t dc_low_limit = this->zero;
  
  // get the point at which the motor moves forward
  while (LEFT_ROTATION == 0.0 && RIGHT_ROTATION == 0.0) {
    this->left_motor.write(dc_high_limit);
    this->right_motor.write(dc_high_limit);
    if (dc_high_limit > (this->zero + DC_CALIBRATION_LIMIT)) {
      DC_MOTOR_ERROR.activate();
      //DC_MOTOR_ERROR.report();
      break;
    }
    dc_high_limit++;
    delay(100);
  }
  
  // reset the conditions
  this->left_motor.write(this->zero);
  this->right_motor.write(this->zero);
  delay(500);
  LEFT_ROTATION = 0.0;
  RIGHT_ROTATION = 0.0;
  
  // get the point at which the motors move backward
  while (LEFT_ROTATION == 0.0 && RIGHT_ROTATION == 0.0) {
    this->left_motor.write(dc_low_limit);
    this->right_motor.write(dc_low_limit);
    if (dc_low_limit < (this->zero - DC_CALIBRATION_LIMIT)) {
      DC_MOTOR_ERROR.activate();
      //DC_MOTOR_ERROR.report();
      break;
    }
    dc_low_limit--;
    delay(100);
  }
  
  this->zero = (dc_high_limit + dc_low_limit) / 2;
  this->left_motor.write(this->zero);
  this->right_motor.write(this->zero);
}

void DCMotor::drive(int8_t motor_speed, int8_t motor_rotation=0) {
  // 
  
  if (motor_speed > 90) {
    motor_speed = 90;
  }
  else if (motor_speed < -90) {
    motor_speed = -90;
  }
  if (motor_rotation > 90) {
    motor_rotation = 90;
  }
  else if (motor_rotation < -90) {
    motor_rotation = -90;
  }
  
  int16_t left_drive = this->zero - motor_speed + motor_rotation;  // subtract motor speed due to upside-down chassis
  int16_t right_drive = this->zero - motor_speed - motor_rotation;
  
  if (left_drive < 0) {
    left_drive = 0;
    right_drive = (2*motor_rotation);
  }
  else if (left_drive > 180) {
    left_drive = 180;
    right_drive = 180 - (2*motor_rotation);
  }

  else if (right_drive < 0) {
    right_drive = 0;
    left_drive = (2*motor_rotation);
  }
  else if (right_drive > 180) {
    right_drive = 180;
    left_drive = 180 - (2*motor_rotation);
  }
  
  if (left_drive > 90) {
    LEFT_DIR = DC_FORWARD;
  } else {
    LEFT_DIR = DC_BACKWARD;
  }
  if (right_drive > 90) {
    RIGHT_DIR = DC_FORWARD;
  } else {
    RIGHT_DIR = DC_BACKWARD;
  }
  
  this->left_motor.write(left_drive);
  this->right_motor.write(right_drive);
  
}

///////////////////////////////////
/// SERVO MOTOR CLASS FUNCTIONS ///
///////////////////////////////////
void ServoMotor::initialize(uint8_t init_port) {
  this->servo.attach(init_port);
}

void ServoMotor::rotate(int16_t servo_position) {
  this->servo.write(servo_position);
}

/////////////////////////////////////
/// STEPPER MOTOR CLASS FUNCTIONS ///
/////////////////////////////////////
void StepperMotor::initialize(uint8_t init_step_pin, uint8_t init_dir_pin, int16_t init_steps_per_rev) {
  this->step_pin = init_step_pin;
  this->dir_pin = init_dir_pin;
  this->steps_per_rev = init_steps_per_rev;
  pinMode(init_step_pin, OUTPUT);
  pinMode(init_dir_pin, OUTPUT);
}

void StepperMotor::rotate(int16_t num_degrees) {
  uint32_t steps_left = abs(((int32_t)num_degrees * this->steps_per_rev) / 360);
  
  if (num_degrees > 0) {
    digitalWrite(this->dir_pin,LOW);
  } else {
    digitalWrite(this->dir_pin,HIGH);
  }
  
  for (steps_left; steps_left > 0; steps_left--) {
    digitalWrite(this->step_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(this->step_pin, HIGH);
    delay(1);
  }
}


void extend_magnets(void) {
  if (!is_extended) {
    uint32_t steps_left = 520; // (180 * steps_per_rev) / 360)
    digitalWrite(STEPPER1.dir_pin, LOW);
    digitalWrite(STEPPER2.dir_pin, HIGH);
    
    for (steps_left; steps_left > 0; steps_left--) {
      digitalWrite(STEPPER1.step_pin, LOW);
      digitalWrite(STEPPER2.step_pin, LOW);
      delayMicroseconds(2);
      digitalWrite(STEPPER1.step_pin, HIGH);
      digitalWrite(STEPPER2.step_pin, HIGH);
      delay(1);
    }
    is_extended = true;
  }
}

void retract_magnets(void) {
  if (is_extended) {
    uint32_t steps_left = 520; // (180 * steps_per_rev) / 360)
    digitalWrite(STEPPER1.dir_pin, LOW);
    digitalWrite(STEPPER2.dir_pin, HIGH);
    
    for (steps_left; steps_left > 0; steps_left--) {
      digitalWrite(STEPPER1.step_pin, LOW);
      digitalWrite(STEPPER2.step_pin, LOW);
      delayMicroseconds(2);
      digitalWrite(STEPPER1.step_pin, HIGH);
      digitalWrite(STEPPER2.step_pin, HIGH);
      delay(1);
    }
    is_extended = false;
  }
}


void toggle_magnets(void) {
  if (is_extended) {
    retract_magnets();
  } else {
    extend_magnets();
  }
}

