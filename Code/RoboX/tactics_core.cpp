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
int wall = 0;

/////////////////
/// FUNCTIONS ///
/////////////////
void init_tactics_core(void) {
  PRINT("\tTactics...");
  Timer3.initialize();  // in microseconds, 1 second by default
  PRINTLN("done");
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
  uint32_t initial_time = millis();
  
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
    if (millis() - initial_time > IDLE_TIMEOUT) {
      OPERATION_MODE = PRIMARY_MODE;
    }
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
  CartVec cart_target = {0, 0};
  PolarVec polar_target = {0, 0};
  char serial_byte = '\0';
  
  SERVO_COLOUR = LED_BLUE;
  
  while(OPERATION_MODE == SECONDARY_MODE) {
    
    switch (operation_state) {
      case SEARCHING:
        if (weight_detect() == true) {
          operation_state = COLLECTING;
          SERVO_COLOUR = LED_GREEN;
        } 
        else {
          cart_target = get_local_target();
        }
        break;
      case COLLECTING:
        
        if (weight_detect() == false) {
          operation_state = SEARCHING;
          SERVO_COLOUR = LED_BLUE;
        } 
        else {
          cart_target = weight_location;
        }
        break;
      case RETURNING:
        cart_target = get_local_target();
        break;
      
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

    if (weight_detect() == true) {
      SERVO_COLOUR = LED_GREEN;
      point_towards_target(weight_location.polar());
    } else {
      SERVO_COLOUR = LED_BLUE;
      Herkulex.moveOneAngle(SMART_SERVO1_ADDRESS, 0, 200, SERVO_COLOUR);
    }
        
    //debug_sensors();
    
    //PRINTLN(LEFT_ROTATION*0.104719755);
    //PRINTLN(RIGHT_ROTATION*0.104719755);
    //PRINTLN();
    COLOUR.update();
    COLOUR.read();
    //delay(200);
    
  }
  PRINTLN("\nReceiving control\n");
  Timer3.detachInterrupt();
}
