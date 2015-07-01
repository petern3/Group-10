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
#include "misc_core.h"


/// GLOBALS ///
//extern float TAU = (2*PI);
static float ENCODER_INCREMENT = (TAU / ENCODER_PPR);
static float ENCODER_MAX = (ENCODER_WRAP * TAU);

static Servo LEFT_DRIVE;
static Servo RIGHT_DRIVE;
static bool LEFT_DIR = DC_FORWARD;
static bool RIGHT_DIR = DC_FORWARD;

extern volatile float LEFT_ROTATION = 0;
extern volatile float RIGHT_ROTATION = 0;

Servo SERVO_1;
Servo SERVO_2;


/// INTERRUPTS ///
static void left_encoder_ISR(void) {
  if (LEFT_DIR == DC_FORWARD) {
    LEFT_ROTATION += ENCODER_INCREMENT;
  } else {
    LEFT_ROTATION -= ENCODER_INCREMENT;
  }
  if (LEFT_ROTATION < -ENCODER_MAX) {
    LEFT_ROTATION = ENCODER_MAX;
  }
  else if (LEFT_ROTATION > ENCODER_MAX) {
    LEFT_ROTATION = -ENCODER_MAX;
  }
}

static void right_encoder_ISR(void) {
  if (RIGHT_DIR == DC_FORWARD) {
    RIGHT_ROTATION += ENCODER_INCREMENT;
  } else {
    RIGHT_ROTATION -= ENCODER_INCREMENT;
  }
  if (RIGHT_ROTATION < -ENCODER_MAX) {
    RIGHT_ROTATION = ENCODER_MAX;
  }
  else if (RIGHT_ROTATION > ENCODER_MAX) {
    RIGHT_ROTATION = -ENCODER_MAX;
  }
}


/// FUNCTIONS ///
void init_actuator_core(void) {
  PRINT("\tActuators...");
  
  // DC Motors
  LEFT_DRIVE.attach(DC_LEFT_PIN);
  RIGHT_DRIVE.attach(DC_RIGHT_PIN);
  attachInterrupt(DC_LEFT_INTERRUPT, left_encoder_ISR, CHANGE);
  attachInterrupt(DC_RIGHT_INTERRUPT, right_encoder_ISR, CHANGE);
  
  // Servos
  SERVO_1.attach(SERVO1_PIN);
  SERVO_2.attach(SERVO2_PIN);
  
  // Smart Servos
  Herkulex.beginSerial2(115200);  // When in port C2 for Transmit/Receive #2
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
  
  int16_t left_drive = 90 + motor_speed + motor_rotation;
  int16_t right_drive = 90 + motor_speed - motor_rotation;
  
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
