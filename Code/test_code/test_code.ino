

#include <Servo.h>

/// PINS ///
#define DC_LEFT_PIN 12
#define DC_RIGHT_PIN 13
#define STEPPER1_STEP_PIN 30
#define STEPPER1_DIR_PIN 31
#define STEPPER2_STEP_PIN 32
#define STEPPER2_DIR_PIN 33
#define STEPPER3_STEP_PIN 34
#define STEPPER3_DIR_PIN 35
#define STEPPER4_STEP_PIN 36
#define STEPPER4_DIR_PIN 37

/// Other config ///
#define STEPPER1_SPR 200
//#define STEPPER1_INV
#define STEPPER2_SPR 200
//#define STEPPER2_INV
#define STEPPER3_SPR 1040  // Geared
//#define STEPPER3_INV
#define STEPPER4_SPR 1000  // Geared
//#define STEPPER4_INV

#define DC_FORWARD true
#define DC_BACKWARD false

#define FORWARD_INCREMENT 10
#define TURNING_INCREMENT 10

#define FWD 'w'
#define BWD 's'
#define LFT 'a'
#define RHT 'd'
#define STP 'x'
#define STEPFWD 'r'
#define STEPBWD 'f'

#define PRINT(s) Serial.print(s)
#define PRINTLN(s) Serial.println(s)

#define STEPPER_1 1
#define STEPPER_2 2
#define STEPPER_3 3
#define STEPPER_4 4


static int8_t FORWARD = 0;
static int8_t TURNING = 0;

Servo SERVO_1;
Servo SERVO_2;
static Servo LEFT_DRIVE;
static Servo RIGHT_DRIVE;
static bool LEFT_DIR = DC_FORWARD;
static bool RIGHT_DIR = DC_FORWARD;

void dc_drive(int8_t motor_speed, int8_t motor_rotation);
void stepper_rotate(uint8_t to_rotate, int16_t num_degrees);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(119200);
  LEFT_DRIVE.attach(DC_LEFT_PIN);
  RIGHT_DRIVE.attach(DC_RIGHT_PIN);
}

void loop() {
  // put your main code here, to run repeatedly:
  PRINTLN("Handing over control:");
  char serial_byte = '\0';
  
  while(true) {
    if (Serial.available() > 0) {
      serial_byte = Serial.read();
      
      if (serial_byte == FWD) {
        FORWARD += FORWARD_INCREMENT;
        if (FORWARD > 90) {
          FORWARD = 90;
        }
        PRINT("\tForward speed value is ");
        PRINTLN(FORWARD);
      }
      else if (serial_byte == BWD) {
        FORWARD -= FORWARD_INCREMENT;
        if (FORWARD < -90) {
          FORWARD = -90;
        }
        PRINT("\tForward speed value is ");
        PRINTLN(FORWARD);
      }
      else if (serial_byte == LFT) {
        TURNING += TURNING_INCREMENT;
        if (TURNING > 90) {
          TURNING = 90;
        }
        PRINT("\tTurning speed value is ");
        PRINTLN(TURNING);
      }
      else if (serial_byte == RHT) {
        TURNING -= TURNING_INCREMENT;
        if (TURNING < -90) {
          TURNING = -90;
        }
        PRINT("\tTurning speed value is ");
        PRINTLN(TURNING);
      }
      else if (serial_byte == STP) {
        FORWARD = 0;
        TURNING = 0;
        PRINTLN("\tStopping");
      }
      else if (serial_byte == STEPFWD) {
        stepper_rotate(STEPPER_1, 180);
        PRINTLN("\tMoving Stepper 1 forward by 180 degrees");
      }
      else if (serial_byte == STEPBWD) {
        stepper_rotate(STEPPER_1, -180);
        PRINTLN("\tMoving Stepper 1 backward by 180 degrees");
      }
    }
    
    dc_drive(FORWARD, TURNING);
  }
  PRINTLN("Receiving control\n");
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

