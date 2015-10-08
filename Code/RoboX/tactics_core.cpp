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
uint8_t home_base = NO_BASE;


typedef struct {
  CartVec left;
  CartVec right;
} Weight_Detect_t;


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
    uint32_t steps_left = STEPPER1_SPR; // 520; // (180 * steps_per_rev) / 360)
    digitalWrite(STEPPER1.dir_pin, LOW);
    digitalWrite(STEPPER2.dir_pin, HIGH);
    
    for (steps_left; steps_left > 0; steps_left--) {
      digitalWrite(STEPPER1.step_pin, LOW);
      digitalWrite(STEPPER2.step_pin, LOW);
      delayMicroseconds(2);
      digitalWrite(STEPPER1.step_pin, HIGH);
      digitalWrite(STEPPER2.step_pin, HIGH);
      delay(4);
      if (LIMIT_O.is_active()) {
        break;
      }
    }
    is_extended = true;
  }
}

void retract_magnets(void) {
  if (is_extended) {
    uint32_t steps_left = 520; // (180 * steps_per_rev) / 360)
    digitalWrite(STEPPER1.dir_pin, HIGH);
    digitalWrite(STEPPER2.dir_pin, LOW);
    
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
  int8_t motor_speed = 0;
  int8_t motor_rotation = 0;
  
  if (target.r > 255) {
    target.r = 255;
  }
  /*if (angle > 180) {
    angle = -angle + 180;
  }*/
  if (angle > BACKING_ANGLE) {
    motor_speed = SPEED_P * -target.r;
    motor_rotation = ROTATE_P * (angle - 180);
  }
  else if (angle < -BACKING_ANGLE) {
    motor_speed = SPEED_P * -target.r;
    motor_rotation = ROTATE_P * (angle + 180);
  }
  else {
    motor_speed = SPEED_P * target.r;
    motor_rotation = ROTATE_P * angle;
  }
  
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

Weight_Detect_t weight_detect(void) {
  Weight_Detect_t found = {{-1, -1}, {-1, -1}};
  
  // Check left sensors (NOT LEFT SIDE). No need for abs as the lower one (USONIC) should always be less.
  if (USONIC1.is_valid()) { 
    if (IR_MED1.polar_read().r - USONIC1.polar_read().r > WEIGHT_DETECT_TOLERANCE ||
        IR_MED1.polar_read().r == NOT_VALID) {
      if (!SONAR1.is_valid() || SONAR1.polar_read().r > 200) {
        found.left = USONIC1.cart_read();
      }
    }
  }
  // Check right sensors
  if (USONIC2.is_valid()) { 
    if (IR_MED2.polar_read().r - USONIC2.polar_read().r > WEIGHT_DETECT_TOLERANCE ||
        IR_MED2.polar_read().r == NOT_VALID) {
      if (!SONAR1.is_valid() || SONAR1.polar_read().r > 200) {
        found.right = USONIC2.cart_read();
      }
    }
  }
  return found;
}

uint8_t colour_detect(void) {
  
  COLOUR.update();
  if (COLOUR.read()[1] > GREEN_THRESHOLD && (COLOUR.read()[1]-GREEN_THRESHOLD) > (COLOUR.read()[2]-BLUE_THRESHOLD)) {  // Green
    return GREEN_BASE;
  }
  else if (COLOUR.read()[2] > BLUE_THRESHOLD && (COLOUR.read()[2]-BLUE_THRESHOLD) > (COLOUR.read()[1]-GREEN_THRESHOLD)) {  // Blue
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
    
    //PRINT("L ("); PRINT(USONIC1.polar_read().cart().x); PRINT(", "); PRINT(USONIC1.polar_read().cart().y); PRINT(") ");
    //PRINT("R ("); PRINT(USONIC2.polar_read().cart().x); PRINT(", "); PRINT(USONIC2.polar_read().cart().y); PRINT(") ");
    
    PRINT("L ("); PRINT(USONIC1.cart_read().x); PRINT(", "); PRINT(USONIC1.cart_read().y); PRINT(") ");
    PRINT("R ("); PRINT(USONIC2.cart_read().x); PRINT(", "); PRINT(USONIC2.cart_read().y); PRINT(") ");
    
    //PRINT(IR_SHT1.polar_read().r); PRINT("  ");
    //PRINT(IR_MED1.polar_read().r); PRINT("  ");
    //PRINT(IR_MED2.polar_read().r); PRINT("  ");
    //PRINT(IR_LNG1.polar_read().r); PRINT("  ");
    //PRINT(IR_LNG2.polar_read().r); PRINT("  ");
    //PRINT(USONIC1.polar_read().r); PRINT("  ");
    //PRINT(USONIC2.polar_read().r); PRINT("  ");
    //PRINT(SONAR1.polar_read().r); PRINT("  ");
    //PRINT(IR_VAR1.read()); PRINT(IR_VAR2.read()); PRINT(IR_VAR3.read());
    PRINT('\r');
  
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
  lower_magnets();
  raise_magnets();
  
  while (OPERATION_MODE == IDLE_MODE) {
    
    OPERATION_MODE = PRIMARY_MODE;
    home_base = colour_detect();
    
    if (home_base == BLUE_BASE) {
      PRINTLN("Blue Team");
    }
    else if (home_base == GREEN_BASE) {
      PRINTLN("Green Team");
    }
    else {
      PRINTLN("No base detected");
    }
    home_base = GREEN_BASE;
    
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
  CartVec left_wall = IR_MED1.cart_read();  // Not confirmed is correct way round
  CartVec right_wall = IR_MED2.cart_read();
  CartVec centre_wall = SONAR1.cart_read();
  CartVec generic_wall = {0, 0};
  
  if (left_wall.x != NOT_VALID && centre_wall.y < CENTRE_SENSOR_TOLERANCE && right_wall.x != NOT_VALID) {  // x  x  x
    if (left_wall.polar().r < centre_wall.polar().r && right_wall.polar().r < centre_wall.polar().r) {
      target.x = 0;
    }
    else if (left_wall.polar().r > right_wall.polar().r) {
      target.x = ROBOT_DIAMETER;
    }
    else {
    target.x = -ROBOT_DIAMETER;
    }
    target.y = -ROBOT_DIAMETER;
    PRINT("x x x");
  }
  else if (left_wall.x == NOT_VALID && centre_wall.y < CENTRE_SENSOR_TOLERANCE && right_wall.x != NOT_VALID) {  // -  x  x
    generic_wall.x = 0;
    generic_wall.y = ROBOT_RADIUS;  //approximation
    target = (centre_wall - generic_wall) + (centre_wall - right_wall);
    PRINT("- x x");
  }
  else if (left_wall.x != NOT_VALID && centre_wall.y > CENTRE_SENSOR_TOLERANCE && right_wall.x != NOT_VALID) {  // x  -  x
    target.x = (left_wall.x + right_wall.x)/2;
    target.y = ROBOT_RADIUS;
    PRINT("x - x");
  }
  else if (left_wall.x != NOT_VALID && centre_wall.y < CENTRE_SENSOR_TOLERANCE && right_wall.x == NOT_VALID) {  // x  x  -
    generic_wall.x = 0;
    generic_wall.y = ROBOT_RADIUS;  //approximation
    target = (centre_wall - generic_wall) + (centre_wall - left_wall);
    PRINT("x x -");
  }
  else if (left_wall.x != NOT_VALID && centre_wall.y > CENTRE_SENSOR_TOLERANCE && right_wall.x == NOT_VALID) {  // x  -  -
    target.x = left_wall.x - ROBOT_RADIUS;
    PRINT("x - -");
  }
  else if (left_wall.x == NOT_VALID && centre_wall.y < CENTRE_SENSOR_TOLERANCE && right_wall.x == NOT_VALID) {  // -  x  -
    target.x = ROBOT_RADIUS;
    target.y = 0;
    PRINT("- x -");
  }
  else if (left_wall.x == NOT_VALID && centre_wall.y > CENTRE_SENSOR_TOLERANCE && right_wall.x != NOT_VALID) {  // -  -  x
    target.x = right_wall.x + ROBOT_RADIUS;
    PRINT("- - x");
  }
  else if (left_wall.x == NOT_VALID && centre_wall.y > CENTRE_SENSOR_TOLERANCE && right_wall.x == NOT_VALID) {  // -  -  -
    target.x = 0;
    target.y = ROBOT_DIAMETER;
    PRINT("- - -  ");
  }
  PRINT(centre_wall.polar().r);
  PRINT("\r");
  /*// x value WIDTH
  if (left_wall.x != NOT_VALID && right_wall.x == NOT_VALID) { // left wall found, not right wall
    target.x = left_wall.x + ROBOT_RADIUS;
  }
  else if (left_wall.x == NOT_VALID && right_wall.x != NOT_VALID) { // right wall found, not left wall
    target.x = right_wall.x - ROBOT_RADIUS;
  } 
  else if (left_wall.x == NOT_VALID && right_wall.x == NOT_VALID) { // no wall found
    target.x = 0 ;
  }
  else {  // If both walls found
    if (centre_wall.y < 200){ // sonar picks up front wall
      if (right_wall.polar().r < left_wall.polar().r) {
        target.x = -ROBOT_RADIUS;
      }
      else {
        target.x = ROBOT_RADIUS;
      }
    }
    else { // drives though gap
      target.x = (left_wall.x + right_wall.x)/2;
    }
  }
  
  // y value LENGTH
  if (centre_wall.y == NOT_VALID) {
    if (left_wall.y > ROBOT_RADIUS && right_wall.y > ROBOT_RADIUS) {
      target.y = min(left_wall.x, right_wall.x) - ROBOT_RADIUS;
    }
    else {
      target.y = ROBOT_DIAMETER;
    }
  }
  else {
    target.y = centre_wall.y - ROBOT_RADIUS;
  }*/
  
  // Backup if not trying to move
  if (target.polar().r < 50) {
    target.y = -ROBOT_DIAMETER;
  }
  // Backup if not actually moving
  /*if (((IMU.read()[0] + IMU.read()[1]) < 50) && ((IMU.read()[3] + IMU.read()[4]) < 50)) {
    target.x = 0;
    target.y = -ROBOT_DIAMETER;
  }*/
  
  return target;
}



void secondary_tactic(void) {
  PRINTLN("Starting secondary tactic:");
  Timer3.setPeriod(SECONDARY_TACTIC_PERIOD);
  Timer3.attachInterrupt(secondary_tactic_ISR);
  
  uint8_t operation_state = SEARCHING;
  int16_t last_weight_time = 0;
  uint16_t weight_timeout = 0;
  
  Weight_Detect_t weight_locations = {{-1, -1}, {-1, -1}};
  uint8_t curr_base = home_base;
  
  CartVec cart_target = {0, 0};
  PolarVec polar_target = {0, 0};
  
  char serial_byte = '\0';
  bool enable_drive = true;
  
  SERVO_COLOUR = LED_WHITE;
  
  while(OPERATION_MODE == SECONDARY_MODE) {
    
    weight_locations = weight_detect();
    
    curr_base = colour_detect();
    IMU.update();
    
    /// Check how many weights I've got ///
    if (is_full()) {
      operation_state = RETURNING;
    }
    
    /// Check if I'm in a base ///
    if (curr_base == home_base && home_base != NO_BASE) {  // In home base
      raise_magnets();
      retract_magnets();
      // Something else?
      // reverse a little
      cart_target.x =  0;
      cart_target.y = -ROBOT_RADIUS;  // get_local_target();
      operation_state = SEARCHING;
    }
    else if (curr_base == NO_BASE) {  // In arena
      extend_magnets();
      switch (operation_state) {
        case SEARCHING:
          SERVO_COLOUR = LED_WHITE; //white doesnt show up :(
          cart_target = get_local_target(); // drive around
          if (weight_locations.left != NO_WEIGHT || weight_locations.right != NO_WEIGHT) { // If I see a weight
            operation_state = COLLECTING;
            last_weight_time = millis();
          }
          break;
        case COLLECTING:
          SERVO_COLOUR = LED_GREEN;
          lower_magnets();
          
          if (weight_locations.left != NO_WEIGHT || weight_locations.right != NO_WEIGHT) { // If I see a weight
            last_weight_time = millis();
            weight_timeout += WEIGHT_TIMEOUT_INC;
            if (weight_timeout > WEIGHT_TIMEOUT_MAX) {
              weight_timeout = WEIGHT_TIMEOUT_MAX;
            }
            if (weight_locations.right.polar() == NO_WEIGHT) {  // If I see left weight
              cart_target = weight_locations.left;
            }
            else if (weight_locations.left.polar() == NO_WEIGHT) {  // If I see right weight
              cart_target = weight_locations.right;
            }
            else {
              if (weight_locations.left.polar().r > weight_locations.right.polar().r) {  // If I see both, choose closest
                cart_target = weight_locations.left;
              }
              else {
                cart_target = weight_locations.right;
              }
            }
            
          }
          else if ((millis() - last_weight_time) > weight_timeout) {  // If lost
            raise_magnets();
            weight_timeout = 0;
            operation_state = SEARCHING;
          }
          // only have the magenets down for max time
          
          
          break;
        case RETURNING:
          SERVO_COLOUR = LED_BLUE;
          raise_magnets();
          cart_target = get_local_target();
          if (!is_full()) {
            operation_state = SEARCHING;
          }
          break;
      }
    }
    else {  // In opposition_base
      SERVO_COLOUR = LED_CYAN;
      raise_magnets();
      cart_target = get_local_target();
    }
    
    /// Perform tasks ///
    polar_target = cart_target.polar();
    point_towards_target(polar_target);
    if (enable_drive == false) {
      polar_target.r = 0;
      polar_target.theta = 0;
    }
    move_towards_target(polar_target);
    
    //PRINTLN(weight_timeout);
    
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
      else if (serial_byte == STP) {
        if (enable_drive == true) {
          enable_drive = false;
          PRINTLN("\tStopping");
        } else {
          enable_drive = true;
          PRINTLN("\tStarting");
        }
      }
    }
    #endif
    
    //PRINT("P ("); PRINT(polar_target.r); PRINT(", "); PRINT(radians_to_degrees(polar_target.theta)); PRINT(") ");
    //PRINT("C ("); PRINT(cart_target.x); PRINT(", "); PRINT(cart_target.y); PRINT(") ");
    
    //debug_sensors();
    
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

  Weight_Detect_t weight_locations = {{-1, -1}, {-1, -1}};
  
  /*Herkulex.moveOneAngle(SMART_SERVO1_ADDRESS, 0, 200, LED_GREEN);
  delay(500);
  Herkulex.moveOneAngle(SMART_SERVO1_ADDRESS, 0, 200, LED_BLUE);
  delay(500);
  Herkulex.moveOneAngle(SMART_SERVO1_ADDRESS, 0, 200, LED_CYAN);
  delay(500);
  Herkulex.moveOneAngle(SMART_SERVO1_ADDRESS, 0, 200, LED_RED);
  delay(500);
  Herkulex.moveOneAngle(SMART_SERVO1_ADDRESS, 0, 200, LED_GREEN2);
  delay(500);
  Herkulex.moveOneAngle(SMART_SERVO1_ADDRESS, 0, 200, LED_PINK);
  delay(500);
  Herkulex.moveOneAngle(SMART_SERVO1_ADDRESS, 0, 200, LED_WHITE);
  delay(500);*/
  
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
    
    /// Base detection ///
    if (colour_detect() == BLUE_BASE) {
      SERVO_COLOUR = LED_BLUE;
    }
    else if (colour_detect() == GREEN_BASE) {
      SERVO_COLOUR = LED_GREEN;
    }
    else {
      SERVO_COLOUR = LED_WHITE;
    }
    
    /// Weight detection ///
    weight_locations = weight_detect();
    //PRINT(weight_locations.left.x); PRINT(", "); PRINT(weight_locations.left.y); PRINT("  ");
    //PRINT(weight_locations.right.x); PRINT(", "); PRINT(weight_locations.right.y); PRINT("\n");
    
    if (weight_locations.left != NO_WEIGHT && weight_locations.right != NO_WEIGHT) {  // If I see two weights
      SERVO_COLOUR = LED_CYAN;
      if (weight_locations.left.polar().r < weight_locations.right.polar().r) {
        point_towards_target(weight_locations.left.polar());
        //PRINT("left");
      }
      else {
        point_towards_target(weight_locations.right.polar());
        //PRINT("right");
      }
    }
    else if (weight_locations.left != NO_WEIGHT && weight_locations.right == NO_WEIGHT) {  // If I see left weight
      SERVO_COLOUR = LED_CYAN;
      point_towards_target(weight_locations.left.polar());
    }
    else if (weight_locations.left == NO_WEIGHT && weight_locations.right != NO_WEIGHT) {  // If I see right weight
      SERVO_COLOUR = LED_CYAN;
      point_towards_target(weight_locations.right.polar());
    }
    else {
      Herkulex.moveOneAngle(SMART_SERVO1_ADDRESS, 0, 200, SERVO_COLOUR);
    }
    
    //debug_sensors();
    
    //PRINTLN(LEFT_ROTATION*0.104719755);
    //PRINTLN(RIGHT_ROTATION*0.104719755);
    //PRINTLN();
    COLOUR.update();
    IMU.update();
    //PRINT(IR_VAR1.is_active()); PRINT(IR_VAR2.is_active()); PRINT(IR_VAR3.is_active()); PRINT('\r');
    //print_buffer(IMU.read(), IMU_BUFFER_SIZE); PRINT('\r');
    //COLOUR.read();
    //delay(200);
    
  }
  PRINTLN("\nReceiving control\n");
  Timer3.detachInterrupt();
}
