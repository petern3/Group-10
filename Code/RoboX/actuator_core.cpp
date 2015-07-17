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
#include "interrupt_core.h"
#include "misc_core.h"


/// GLOBALS ///
static Servo LEFT_DRIVE;
static Servo RIGHT_DRIVE;
static uint8_t DC_STOP = 90;

Servo SERVO_1;
Servo SERVO_2;


/// FUNCTIONS ///
void dc_calibrate(void) {
  LEFT_DRIVE.write(DC_STOP);
  RIGHT_DRIVE.write(DC_STOP);
  uint8_t dc_high_limit = DC_STOP;
  uint8_t dc_low_limit = DC_STOP;
  
  // get the point at which the motor moves forward
  while (LEFT_ROTATION == 0.0 && RIGHT_ROTATION == 0.0) {
    LEFT_DRIVE.write(dc_high_limit);
    RIGHT_DRIVE.write(dc_high_limit);
    if (dc_high_limit > (DC_STOP + DC_CALIBRATION_LIMIT)) {
      break;
    }
    dc_high_limit++;
    delay(100);
  }
  
  // reset the conditions
  LEFT_DRIVE.write(DC_STOP);
  RIGHT_DRIVE.write(DC_STOP);
  delay(500);
  LEFT_ROTATION = 0.0;
  RIGHT_ROTATION = 0.0;
  
  // get the point at which the motors move backward
  while (LEFT_ROTATION == 0.0 && RIGHT_ROTATION == 0.0) {
    LEFT_DRIVE.write(dc_low_limit);
    RIGHT_DRIVE.write(dc_low_limit);
    if (dc_low_limit < (DC_STOP - DC_CALIBRATION_LIMIT)) {
      break;
    }
    dc_low_limit--;
    delay(100);
  }
  
  DC_STOP = (dc_high_limit + dc_low_limit) / 2;
  PRINT(DC_STOP);
  LEFT_DRIVE.write(DC_STOP);
  RIGHT_DRIVE.write(DC_STOP);
}


void init_actuator_core(void) {
  PRINT("\tDC Motors...");
  
  // DC Motors
  LEFT_DRIVE.attach(DC_LEFT_PIN);
  RIGHT_DRIVE.attach(DC_RIGHT_PIN);
  attachInterrupt(DC_LEFT_INTERRUPT_PIN, left_encoder_ISR, RISING);  // can do 'CHANGE', but would include twice the interrupts (high CPU load)
  attachInterrupt(DC_RIGHT_INTERRUPT_PIN, right_encoder_ISR, RISING);
  dc_calibrate();
  
  PRINTLN("done");
  PRINT("\tActuators...");
  // Servos
  SERVO_1.attach(SERVO1_PIN);
  SERVO_2.attach(SERVO2_PIN);
  
  // Smart Servos
  //Herkulex.beginSerial2(115200);  // When in port C2 for Transmit/Receive #2
  //Herkulex.reboot(SMART_SERVO1_ADDRESS);
  //Herkulex.initialize();
  
  // Stepper Motors
  pinMode(STEPPER1_STEP_PIN,OUTPUT);
  pinMode(STEPPER1_DIR_PIN,OUTPUT);
  pinMode(STEPPER2_STEP_PIN,OUTPUT);
  pinMode(STEPPER2_DIR_PIN,OUTPUT);
  pinMode(STEPPER3_STEP_PIN,OUTPUT);
  pinMode(STEPPER3_DIR_PIN,OUTPUT);
  pinMode(STEPPER4_STEP_PIN,OUTPUT);
  pinMode(STEPPER4_DIR_PIN,OUTPUT);
  
  PRINTLN("done");
}


void dc_drive(int8_t motor_speed, int8_t motor_rotation=0) {
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
  
  int16_t left_drive = DC_STOP - motor_speed + motor_rotation;  // subtract motor speed due to upside-down chassis
  int16_t right_drive = DC_STOP - motor_speed - motor_rotation;
  
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
  
  LEFT_DRIVE.write(left_drive);
  RIGHT_DRIVE.write(right_drive);
  
}


void servo_rotate(Servo to_rotate, uint8_t servo_position) {
  if (servo_position < 180) {
    to_rotate.write(servo_position);
  } else {
    to_rotate.write(180);
  }
}


static void stepper1_rotate(int16_t num_degrees) {
  uint32_t steps_left = abs(((int32_t)num_degrees * STEPPER1_SPR) / 360);
  
  #ifndef STEPPER1_INV
    if (num_degrees > 0) {
      digitalWrite(STEPPER1_DIR_PIN,HIGH);
    } else {
      digitalWrite(STEPPER1_DIR_PIN,LOW);
    }
  #else
    if (num_degrees > 0) {
      digitalWrite(STEPPER1_DIR_PIN,LOW);
    } else {
      digitalWrite(STEPPER1_DIR_PIN,HIGH);
    }
  #endif
  
  for (steps_left; steps_left > 0; steps_left--) {
    digitalWrite(STEPPER1_STEP_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(STEPPER1_STEP_PIN, HIGH);
    delay(1);
  }
}

static void stepper2_rotate(int16_t num_degrees) {
  uint32_t steps_left = abs(((int32_t)num_degrees * STEPPER2_SPR) / 360);
  
  #ifndef STEPPER2_INV
    if (num_degrees > 0) {
      digitalWrite(STEPPER2_DIR_PIN,HIGH);
    } else {
      digitalWrite(STEPPER2_DIR_PIN,LOW);
    }
  #else
    if (num_degrees > 0) {
      digitalWrite(STEPPER2_DIR_PIN,LOW);
    } else {
      digitalWrite(STEPPER2_DIR_PIN,HIGH);
    }
  #endif
  
  for (steps_left; steps_left > 0; steps_left--) {
    digitalWrite(STEPPER2_STEP_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(STEPPER2_STEP_PIN, HIGH);
    delay(1);
  }
}

static void stepper3_rotate(int16_t num_degrees) {
  uint32_t steps_left = abs(((int32_t)num_degrees * STEPPER3_SPR) / 360);
  
  #ifndef STEPPER3_INV
    if (num_degrees > 0) {
      digitalWrite(STEPPER3_DIR_PIN,HIGH);
    } else {
      digitalWrite(STEPPER3_DIR_PIN,LOW);
    }
  #else
    if (num_degrees > 0) {
      digitalWrite(STEPPER3_DIR_PIN,LOW);
    } else {
      digitalWrite(STEPPER3_DIR_PIN,HIGH);
    }
  #endif
  
  for (steps_left; steps_left > 0; steps_left--) {
    digitalWrite(STEPPER3_STEP_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(STEPPER3_STEP_PIN, HIGH);
    delay(1);
  }
}

static void stepper4_rotate(int16_t num_degrees) {
  uint32_t steps_left = abs(((int32_t)num_degrees * STEPPER4_SPR) / 360);
  
  #ifndef STEPPER4_INV
    if (num_degrees > 0) {
      digitalWrite(STEPPER4_DIR_PIN,HIGH);
    } else {
      digitalWrite(STEPPER4_DIR_PIN,LOW);
    }
  #else
    if (num_degrees > 0) {
      digitalWrite(STEPPER4_DIR_PIN,LOW);
    } else {
      digitalWrite(STEPPER4_DIR_PIN,HIGH);
    }
  #endif
  
  for (steps_left; steps_left > 0; steps_left--) {
    digitalWrite(STEPPER4_STEP_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(STEPPER4_STEP_PIN, HIGH);
    delay(1);
  }
}


void stepper_rotate(uint8_t to_rotate, int16_t num_degrees) {
  if (to_rotate == STEPPER_1) {
    stepper1_rotate(num_degrees);
  }
  else if (to_rotate == STEPPER_2) {
    stepper2_rotate(num_degrees);
  }
  else if (to_rotate == STEPPER_3) {
    stepper3_rotate(num_degrees);
  }
  else if (to_rotate == STEPPER_4) {
    stepper4_rotate(num_degrees);
  }
}

