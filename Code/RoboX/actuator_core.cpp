 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * actuator_core.cpp
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-31
 * Edited:  2015-05-31
 * 
 * Blurb
 * 
 *//////////////////////////////////////////////////////////////////////


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
  
  // Smart Servos
  Herkulex.beginSerial2(115200);  // When in port C2
  Herkulex.reboot(SMART_SERVO1_ADDRESS);
  Herkulex.initialize();
  
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
  if (to_rotate == STEPPER1) {
    stepper1_rotate(num_degrees);
  }
  else if (to_rotate == STEPPER2) {
    stepper2_rotate(num_degrees);
  }
  else if (to_rotate == STEPPER3) {
    stepper3_rotate(num_degrees);
  }
  else if (to_rotate == STEPPER4) {
    stepper4_rotate(num_degrees);
  }
}
