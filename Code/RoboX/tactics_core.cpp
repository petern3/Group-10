 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * tactics_core.cpp
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-06-17
 * Edited:  2015-06-18
 * 
 * Blurb
 * 
 *//////////////////////////////////////////////////////////////////////

////////////////
/// INCLUDES ///
////////////////
#include "tactics_core.h"
#include "actuator_core.h"
#include "exception_core.h"
#include "interrupt_core.h"
#include "map_core.h"
#include "trig_core.h"
#include "sensor_core.h"

///////////////
/// GLOBALS ///
///////////////
uint8_t OPERATION_MODE = IDLE_MODE;

//CartVec weight_location = {-1, -1};
int8_t NO_WEIGHT[2] = {-1, -1};

bool is_extended = true;
bool is_lowered = false;

int wall = 0;

/////////////////
/// FUNCTIONS ///
/////////////////
void init_tactics_core(void) {
  PRINT("\tTactics...");
  Timer3.initialize();  // in microseconds, 1 second by default
  PRINTLN("done");
}


/// ACTUATORS ///

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
      delay(4);
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
      delay(4);
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

void raise_magnets(void) {
  if (is_lowered) {
    SERVO1.rotate(MAX_TRAVEL);
    SERVO2.rotate(0);
    is_lowered = false;
  }
}

void lower_magnets(void) {
  if (!is_lowered) {
    SERVO1.rotate(0);
    SERVO2.rotate(MAX_TRAVEL);
    is_lowered = true;
  }
}


static void move_towards_target(PolarVec target) {
  int16_t angle = radians_to_degrees(target.theta);
  
  if (target.r > 255) {
    target.r = 255;
  }
  if (angle > 180) {
    angle = -angle + 180;
  }
  /*
  if (angle > 90) {
    if (!IR_LNG1.is_valid() || IR_LNG1.polar_read().r > 30) {
      weight_location = USONIC1.cart_read();
      return true;
    }
  }*/
  int8_t motor_speed = SPEED_P * target.r;
  int8_t motor_rotation = ROTATE_P * angle;
  
  DC.drive(motor_speed, motor_rotation);
}

static void point_towards_target(PolarVec target) {
  int16_t angle = radians_to_degrees(target.theta);
  if (angle > 150) {
    Herkulex.moveOneAngle(SMART_SERVO1_ADDRESS, angle-180, 200, LED_RED);
  }
  else if (angle < -150) {
    Herkulex.moveOneAngle(SMART_SERVO1_ADDRESS, angle+180, 200, LED_RED);
  }
  else {
    Herkulex.moveOneAngle(SMART_SERVO1_ADDRESS, angle, 200, SERVO_COLOUR);
  }
}


/// SENSORS ///

CartVec* weight_detect(void) {
  CartVec found[2] = {{-1, -1}, {-1, -1}};
  
  // Check right. No need for abs as the lower one (USONIC) should always be less.
  if (USONIC1.is_valid()) { 
    if (IR_MED1.polar_read().r - USONIC1.polar_read().r > WEIGHT_DETECT_TOLERANCE ||
        IR_MED1.polar_read().r == NOT_VALID) {
       if (!IR_LNG1.is_valid() || IR_LNG1.polar_read().r > 400) {
        found[0] = USONIC1.cart_read();
       }
    }
  }
  // Check left
  if (USONIC2.is_valid()) { 
    if (IR_MED2.polar_read().r - USONIC2.polar_read().r > WEIGHT_DETECT_TOLERANCE ||
        IR_MED1.polar_read().r == NOT_VALID) {
      if (!IR_LNG1.is_valid() || IR_LNG1.polar_read().r > 400) {
        found[1] = USONIC1.cart_read();
       }
    }
  }
  return found;
}

uint8_t colour_detect(void) {
  
  COLOUR.update();
  if (COLOUR.read()[1] > GREEN_THRESHOLD) {  // Green
    return GREEN_BASE;
  }
  else if (COLOUR.read()[2] > BLUE_THRESHOLD) {  // Blue
    return BLUE_BASE;
  }
  else {
    return NO_BASE;
  }
}

bool is_full(void) {
  if (IR_VAR1.is_active() && IR_VAR2.is_active() && IR_VAR3.is_active()) {
    return true;
  }
  return false;
}

static void debug_sensors(void) {
    
    PRINT("L ("); PRINT(USONIC1.polar_read().cart().x); PRINT(", "); PRINT(USONIC1.polar_read().cart().y); PRINT(") ");
    PRINT("R ("); PRINT(USONIC2.polar_read().cart().x); PRINT(", "); PRINT(USONIC2.polar_read().cart().y); PRINT(") ");
    
    PRINT("L ("); PRINT(USONIC1.cart_read().x); PRINT(", "); PRINT(USONIC1.cart_read().y); PRINT(") ");
    PRINT("R ("); PRINT(USONIC2.cart_read().x); PRINT(", "); PRINT(USONIC2.cart_read().y); PRINT(") ");
    
    //PRINT(IR_SHT1.polar_read().r); PRINT("  ");
    PRINT(IR_MED1.polar_read().r); PRINT("  ");
    PRINT(IR_MED2.polar_read().r); PRINT("  ");
    PRINT(IR_LNG1.polar_read().r); PRINT("  ");
    //PRINT(IR_LNG2.polar_read().r); PRINT("  ");
    PRINT(USONIC1.polar_read().r); PRINT("  ");
    PRINT(USONIC2.polar_read().r); PRINT("  ");
    //PRINT(IR_VAR1.read()); PRINT(IR_VAR2.read()); PRINT(IR_VAR3.read());
    PRINT('\n');
  
}

static void print_buffer(int16_t* to_print, uint16_t len) {
  for (uint16_t i=0; i < len; i++) {
    PRINT(to_print[i]);
    PRINT("  ");
  }
  PRINTLN();
}


/////////////////
/// IDLE MODE ///
/////////////////
void idle_mode(void) {
  PRINTLN("Idling\n");
  char serial_byte = '\0';
  //uint32_t initial_time = millis();
  OPERATION_MODE = MANUAL_MODE;
  
  while (OPERATION_MODE == IDLE_MODE) {
    
    #ifdef ENABLE_SERIAL
    if (Serial.available() > 0) {
      serial_byte = Serial.read();
      
      if (serial_byte == PRIMARY_MODE) {
        OPERATION_MODE = PRIMARY_MODE;
      }
      else if (serial_byte == SECONDARY_MODE) {
        OPERATION_MODE = SECONDARY_MODE;
      }
      else if (serial_byte == MANUAL_MODE) {
        OPERATION_MODE = MANUAL_MODE;
      }
    }
    #endif
    //if (millis() - initial_time > IDLE_TIMEOUT) {
    //  OPERATION_MODE = PRIMARY_MODE;
    //}
  }
}

//////////////////////
/// PRIMARY TACTIC ///
//////////////////////
void primary_tactic(void) {
  PRINTLN("Starting primary tactic:");
  Timer3.setPeriod(PRIMARY_TACTIC_PERIOD);
  Timer3.attachInterrupt(primary_tactic_ISR);
  
  uint8_t operation_state = SEARCHING;
  Position_t position_target = {0, 0};
  PolarVec polar_target = {0, 0};
  char serial_byte = '\0';
  
  while (!SD_ERROR.is_active && OPERATION_MODE == PRIMARY_MODE) {
    
    
    #ifdef ENABLE_SERIAL
    if (Serial.available() > 0) {
      serial_byte = Serial.read();
      
      if (serial_byte == SECONDARY_MODE) {
        OPERATION_MODE = SECONDARY_MODE;
      }
      else if (serial_byte == MANUAL_MODE) {
        OPERATION_MODE = MANUAL_MODE;
      }
      else if (serial_byte == IDLE_MODE) {
        OPERATION_MODE = IDLE_MODE;
      }
    }
    #endif
  }
  PRINT("\nPrimary tactic ");
  Timer3.detachInterrupt();
  if (OPERATION_MODE == PRIMARY_MODE) {
    PRINTLN("failure\n");
    OPERATION_MODE = SECONDARY_MODE;  // if there was an unexpected failure
  } else {
    PRINTLN("ending\n");
  }
}


////////////////////////
/// SECONDARY TACTIC ///
////////////////////////
static CartVec get_local_target(void) {
  CartVec target = {0, 0};
  CartVec left_wall = IR_MED2.cart_read();
  CartVec right_wall = IR_MED1.cart_read();
  CartVec centre_wall = IR_LNG1.cart_read();
  // left wall == 1, right wall == 2
  
  // x value WIDTH
  if (left_wall.x != NOT_VALID && right_wall.x == NOT_VALID) { // left wall found, not right wall
    target.x = left_wall.x + ROBOT_DIAMETER;
    wall = 1;
  }
  else if (left_wall.x == NOT_VALID && right_wall.x != NOT_VALID) { // right wall found, not left wall
    target.x = right_wall.x - ROBOT_DIAMETER;
    wall = 2;
  } 
  else if (left_wall.x == NOT_VALID && right_wall.x == NOT_VALID) { // no wall found
    target.x = 0 ;
  }
  else { // If both walls found
    if (wall == 1){ //left wall
      target.x = 100;  //(left_wall.x + right_wall.x) / 2;
    }
    else if (wall == 2){ //right wall
      target.x = -100;
    }
    //wall = 0;
  }
  
  // y value LENGTH
  if (left_wall.y != NOT_VALID && right_wall.y == NOT_VALID) { // left wall found, not right wall
    target.y = (left_wall.y) - ROBOT_RADIUS;
  }
  else if (left_wall.y == NOT_VALID && right_wall.y != NOT_VALID) { // right wall found, not left wall
    target.y = (right_wall.y) - ROBOT_RADIUS;
  }
  else  if (left_wall.y == NOT_VALID && right_wall.y == NOT_VALID) { // No walls found move fall
    target.y = ROBOT_RADIUS;
  }
  else { // Both walls found / corner
    target.y = 0;
    /*if (centre_wall.y != NOT_VALID) { // Corner found
      target.x = 400;
      target.y = -300;
    }
    else {*/
      //target.y = ((left_wall.y + right_wall.y) / 2) - ROBOT_RADIUS; // Gap found
    //}
  }
  /*if (IR_LNG1.is_valid() && IR_LNG1 < 100) {
    target.
  }*/
  //target.x = -100;
  //target.y = 0;
  return target;
}



void secondary_tactic(void) {
  PRINTLN("Starting secondary tactic:");
  Timer3.setPeriod(SECONDARY_TACTIC_PERIOD);
  Timer3.attachInterrupt(secondary_tactic_ISR);
  
  uint8_t operation_state = SEARCHING;
  int16_t last_weight_time = 0;
  CartVec weight_locations[2] = {{-1, -1}, {-1, -1}};
  
  CartVec cart_target = {0, 0};
  PolarVec polar_target = {0, 0};
  
  char serial_byte = '\0';
  
  SERVO_COLOUR = LED_BLUE;
  
  while(OPERATION_MODE == SECONDARY_MODE) {
    
    weight_locations[0] = weight_detect()[0];
    weight_locations[1] = weight_detect()[1];
    
    switch (operation_state) {
      case SEARCHING:
        if (weight_locations[0] != NO_WEIGHT || weight_locations[1] != NO_WEIGHT) {
          operation_state = COLLECTING;
          SERVO_COLOUR = LED_GREEN;
          last_weight_time = millis();
        }
        else {
          cart_target = get_local_target();
        }
        break;
      case COLLECTING:
        
        if (weight_locations[0] != NO_WEIGHT || weight_locations[1] != NO_WEIGHT) {
          last_weight_time = millis();
          if (weight_locations[0].polar().r < weight_locations[1].polar().r) {
            cart_target = weight_locations[0];
          } else {
            cart_target = weight_locations[1];
          }
          
        }
        else if ((millis() - last_weight_time) > WEIGHT_LOST_TOLERANCE) {
          operation_state = SEARCHING;
          SERVO_COLOUR = LED_BLUE;
        }
        
        break;
      case RETURNING:
        cart_target = get_local_target();
        break;
    }
    if (is_full()) {
      operation_state = RETURNING;
      SERVO_COLOUR = LED_GREEN2;
    }
    
    
    polar_target = cart_target.polar();
    move_towards_target(polar_target);
    point_towards_target(polar_target);
    
    #ifdef ENABLE_SERIAL
    if (Serial.available() > 0) {
      serial_byte = Serial.read();
      
      if (serial_byte == PRIMARY_MODE) {
        OPERATION_MODE = PRIMARY_MODE;
      }
      else if (serial_byte == MANUAL_MODE) {
        OPERATION_MODE = MANUAL_MODE;
      }
      else if (serial_byte == IDLE_MODE) {
        OPERATION_MODE = IDLE_MODE;
      }
    }
    #endif
    
    PRINT("P ("); PRINT(polar_target.r); PRINT(", "); PRINT(radians_to_degrees(polar_target.theta)); PRINT(") ");
    PRINT("C ("); PRINT(cart_target.x); PRINT(", "); PRINT(cart_target.y); PRINT(") ");
    
    debug_sensors();
    
  }
  PRINTLN("\nSecondary tactic ending\n");
  Timer3.detachInterrupt();
}


///////////////////
/// MANUAL MODE ///
///////////////////
void manual_mode(void) {
  PRINTLN("Handing over control:");
  Timer3.setPeriod(MANUAL_CONTROL_PERIOD);
  Timer3.attachInterrupt(manual_control_ISR);
  
  int8_t FORWARD = 0;
  int8_t TURNING = 0;
  char serial_byte = '\0';

  CartVec weight_locations[2] = {{-1, -1}, {-1, -1}};
  
  while(OPERATION_MODE == MANUAL_MODE) {
    #ifdef ENABLE_SERIAL
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
      else if (serial_byte == RHT) {
        TURNING += TURNING_INCREMENT;
        if (TURNING > 90) {
          TURNING = 90;
        }
        PRINT("\tTurning speed value is ");
        PRINTLN(TURNING);
      }
      else if (serial_byte == LFT) {
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
      
      else if (serial_byte == EXTEND) {
        extend_magnets();
        PRINTLN("\tExtending magnets");
      }
      else if (serial_byte == RETRACT) {
        retract_magnets();
        PRINTLN("\tRetracting magnets");
      }
      else if (serial_byte == TOGGLE) {
        toggle_magnets();
        PRINTLN("\tToggling magnets");
      }

      else if (serial_byte == RAISE) {
        raise_magnets();
        PRINTLN("\tRaising magnets");
      }
      else if (serial_byte == LOWER) {
        lower_magnets();
        PRINTLN("\tLowering magnets");
      }
      
      else if (serial_byte == PRIMARY_MODE) {
        OPERATION_MODE = PRIMARY_MODE;
      }
      else if (serial_byte == SECONDARY_MODE) {
        OPERATION_MODE = SECONDARY_MODE;
      }
      else if (serial_byte == IDLE_MODE) {
        OPERATION_MODE = IDLE_MODE;
      }
      
      
      else if (serial_byte == 'q') {
        Herkulex.moveOneAngle(SMART_SERVO1_ADDRESS, 150, 200, LED_BLUE);
        PRINTLN("\tMoving smart servo to 150");
      }
      else if (serial_byte == 'e') {
        Herkulex.moveOneAngle(SMART_SERVO1_ADDRESS, -150, 200, LED_GREEN);
        PRINTLN("\tMoving smart servo to -150");
      }
      else if (serial_byte == 'z') {
        Herkulex.moveOneAngle(SMART_SERVO1_ADDRESS, 0, 200, LED_WHITE);
        PRINTLN("\tMoving smart servo to 0");
      }
      
    }
    
    #endif
    
    DC.drive(FORWARD, TURNING);
    
    weight_locations[0] = weight_detect()[0];
    weight_locations[1] = weight_detect()[1];  // TO CONFIRM
    if (weight_locations[0] != NO_WEIGHT || weight_locations[1] != NO_WEIGHT) {
      SERVO_COLOUR = LED_GREEN;
      if (weight_locations[0].polar().r < weight_locations[1].polar().r) {
        point_towards_target(weight_locations[0].polar());
      } else {
        point_towards_target(weight_locations[1].polar());
      }
    } else {
      SERVO_COLOUR = LED_BLUE;
      Herkulex.moveOneAngle(SMART_SERVO1_ADDRESS, 0, 200, SERVO_COLOUR);
    }
        
    //debug_sensors();
    
    //PRINTLN(LEFT_ROTATION*0.104719755);
    //PRINTLN(RIGHT_ROTATION*0.104719755);
    //PRINTLN();
    COLOUR.update();
    IMU.update();
    PRINT(IR_VAR1.is_active()); PRINT(IR_VAR2.is_active()); PRINT(IR_VAR3.is_active()); PRINT('\r');
    //print_buffer(IMU.read(), IMU_BUFFER_SIZE); PRINT('\r');
    //COLOUR.read();
    //delay(200);
    
  }
  PRINTLN("\nReceiving control\n");
  Timer3.detachInterrupt();
}
