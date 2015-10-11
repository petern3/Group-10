 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * tactics_core.cpp
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls, Jack Hendrikz and Ryan Taylor
 * Created: 2015-06-17
 * Edited:  2015-10-10
 * 
 * Blurb
 * 
 *//////////////////////////////////////////////////////////////////////

////////////////
/// INCLUDES ///
////////////////
#include "tactics_core.h"
#include "actuator_core.h"
#include "audio_core.h"
#include "exception_core.h"
#include "interrupt_core.h"
#include "map_core.h"
#include "trig_core.h"
#include "sensor_core.h"

///////////////
/// GLOBALS ///
///////////////
uint8_t OPERATION_MODE = IDLE_MODE;
//int16_t IMU_time = 0;
//int flag = 0;

//CartVec weight_location = {-1, -1};
int8_t NO_WEIGHT[2] = {-1, -1};

bool is_extended = true;
bool is_lowered = false;
bool stuck_flag = false;
uint8_t home_base = NO_BASE;
CircBuf_t stop_buffer_x;
CircBuf_t stop_buffer_y;


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
  if (!is_extended && DIP8_S4.is_active()) {
    uint32_t steps_left = STEPPER1_SPR; // 520; // (180 * steps_per_rev) / 360)
    digitalWrite(STEPPER1.dir_pin, HIGH);
    digitalWrite(STEPPER2.dir_pin, LOW);
    
    for (steps_left; steps_left > 0; steps_left--) {
      digitalWrite(STEPPER1.step_pin, LOW);
      digitalWrite(STEPPER2.step_pin, LOW);
      delayMicroseconds(2);
      digitalWrite(STEPPER1.step_pin, HIGH);
      digitalWrite(STEPPER2.step_pin, HIGH);
      delay(2);
      if (LIMIT_O.is_active()) {
        break;
      }
    }
    is_extended = true;
  }
}

void retract_magnets(void) {
  if (is_extended && DIP8_S4.is_active()) {
    uint32_t steps_left = 520; // (180 * steps_per_rev) / 360)
    digitalWrite(STEPPER1.dir_pin, LOW);
    digitalWrite(STEPPER2.dir_pin, HIGH);
    
    for (steps_left; steps_left > 0; steps_left--) {
      digitalWrite(STEPPER1.step_pin, LOW);
      digitalWrite(STEPPER2.step_pin, LOW);
      delayMicroseconds(2);
      digitalWrite(STEPPER1.step_pin, HIGH);
      digitalWrite(STEPPER2.step_pin, HIGH);
      delay(2);
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
  if (is_lowered && DIP8_S5.is_active()) {
    SERVO1.rotate(MAX_TRAVEL);
    SERVO2.rotate(0);
    is_lowered = false;
    PRINTLN("raise");
  }
}

void lower_magnets(void) {
  if (!is_lowered && DIP8_S5.is_active()) {
    SERVO1.rotate(0);
    SERVO2.rotate(MAX_TRAVEL);
    is_lowered = true;
    PRINTLN("lower");
  }
}


static void move_towards_target(PolarVec target) {
  int16_t angle = radians_to_degrees(target.theta);
  int16_t motor_speed = 0;
  int16_t motor_rotation = 0;
  
  /*if (target.r > 255) {
    target.r = 255;
  }
  if (angle > 180) {
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

  if (angle < -ANGLE_LIMIT || angle > ANGLE_LIMIT){
  	motor_speed = 0;
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
  	// USONIC 1 is left
  	// USONIC 2 is right
  	// USONIC 3 is upper centre

  	// IR_MED1 left one
  	// IR_MED2 right one
  
  	// Check left sensors (NOT LEFT SIDE). No need for abs as the lower one (USONIC) should always be less.
  	if (USONIC1.is_valid()) { 
    	if ((IR_MED1.polar_read().r - USONIC1.polar_read().r) > WEIGHT_DETECT_TOLERANCE ||
            IR_MED1.polar_read().r == NOT_VALID) { //weight next to wall, WEIGHT_DETECT_TOLERANCE should be the distance the our 
        	//robot can pick a weight up next to a way... not the mesurement of a weight...
      		if (!USONIC3.is_valid() || USONIC3.polar_read().r > CENTRE_SENSOR_TOLERANCE) { // checks to see a poll, this 200 may need ajusting
        		found.left = USONIC1.cart_read();
      		}
    	}
  	}
  	// Check right sensors
  	if (USONIC2.is_valid()) { 
    	if ((IR_MED2.polar_read().r - USONIC2.polar_read().r) > WEIGHT_DETECT_TOLERANCE ||
            IR_MED2.polar_read().r == NOT_VALID) {
      		if (!USONIC3.is_valid() || USONIC3.polar_read().r > CENTRE_SENSOR_TOLERANCE) {
        		found.right = USONIC2.cart_read();
      		}
    	}
  	}
    
  	return found;
}

uint8_t colour_detect(void) {
  
  COLOUR.update();
  /*if (COLOUR.read()[1] > GREEN_THRESHOLD && (COLOUR.read()[1]-GREEN_THRESHOLD) > (COLOUR.read()[2]-BLUE_THRESHOLD)) {  // Green
    return GREEN_BASE;
  }
  else if (COLOUR.read()[2] > BLUE_THRESHOLD && (COLOUR.read()[2]-BLUE_THRESHOLD) > (COLOUR.read()[1]-GREEN_THRESHOLD)) {  // Blue
    return BLUE_BASE;
  }*/
  if (COLOUR.read()[2] > BLUE_THRESHOLD) {  // Blue
    return BLUE_BASE;
  }
  else if (COLOUR.read()[1] > GREEN_THRESHOLD) {  // Green
    return GREEN_BASE;
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
    
    //PRINT("L ("); PRINT(USONIC1.cart_read().x); PRINT(", "); PRINT(USONIC1.cart_read().y); PRINT(") ");
    //PRINT("R ("); PRINT(USONIC2.cart_read().x); PRINT(", "); PRINT(USONIC2.cart_read().y); PRINT(") ");
    
    //PRINT(IR_SHT1.polar_read().r); PRINT("  "); //A5
    PRINT(IR_MED1.polar_read().r); PRINT("  ");// left one
    PRINT(IR_MED2.polar_read().r); PRINT("  ");// right one
    PRINT(IR_LNG1.polar_read().r); PRINT("  ");// used
    //PRINT(IR_LNG2.polar_read().r); PRINT("  ");
    //PRINT(USONIC1.polar_read().r); PRINT("  "); //left
    //PRINT(USONIC2.polar_read().r); PRINT("  "); //right
    //PRINT(USONIC3.polar_read().r); PRINT("  "); //centre
    PRINT(analogRead(A5)); PRINT("  "); // 
    //PRINT(SONAR1.polar_read().r); PRINT("  ");
    //PRINT(IR_VAR1.read()); PRINT(IR_VAR2.read()); PRINT(IR_VAR3.read());
    //PRINT(abs(IMU.read()[0]) + abs(IMU.read()[1])); PRINT("  ");
    //PRINT(IMU.read()[1]); PRINT("  ");
    //PRINT(abs(IR_MED1.polar_read().r) - abs(USONIC1.polar_read().r)); PRINT("  ");
    //PRINT(abs(IR_MED2.polar_read().r) - abs(USONIC2.polar_read().r)); PRINT("  ");
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
  raise_magnets(); //PRINTLN("idle");
  
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
    
    if (DIP8_S1.is_active()) {
      OPERATION_MODE = PRIMARY_MODE;
    }
    else {
      OPERATION_MODE = MANUAL_MODE;
    }
    
    
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
  	CartVec centre_S_IR = IR_SHT1.cart_read();  // left one
  	CartVec left_IR = IR_MED1.cart_read();  // left one
  	CartVec right_IR = IR_MED2.cart_read(); // right one
  	CartVec centre_IR = IR_LNG1.cart_read();
  	CartVec centre_ULTRA = USONIC3.cart_read(); // middle ultrasonic sensor
  
  	if (left_IR.x != NOT_VALID && centre_IR.y != NOT_VALID && right_IR.x != NOT_VALID) {  // x  x  x
      	if (left_IR.x < 150 && right_IR.x < 150){
			if (left_IR.x > right_IR.x && right_IR.y < centre_IR.y < left_IR.y){ //case 3 wall on right
	  			target.x = random(-350,-400);
	  			target.y = 100;
			}
			else if (left_IR.x < right_IR.x && right_IR.y > centre_IR.y > left_IR.y){//case 4 wall on left
	  			target.x = random(350,400);
	  			target.y = 100;
			}
			else if(centre_IR.y > left_IR.y || centre_IR.y > right_IR.y){//case 1 in corner
	  			target.x = 300;//this turns left may need to turn right
	  			target.y = -50;
			}
			else {
				target.x = 0;
				target.y = -300;
			}
      	}
		else {
    		target.x = 0;
			target.y = 130;
    	}
		/*else if (centre_SONAR.y < left_IR.y && centre_ULTRA.y < right_IR.y){// case 5 detects a poll near the wall
	  		target.x = 300;
	  		target.y = -50;
		}*/
    	PRINT("x x x         ");
  	}
  	else if (left_IR.x == NOT_VALID && centre_IR.y != NOT_VALID && right_IR.x != NOT_VALID) {  // x  x  - these are objects on left and centre
    	target.x = ROBOT_RADIUS;
    	target.y = 100;
    	PRINT("x x -         ");
  	}
  	else if (left_IR.x != NOT_VALID && centre_IR.y == NOT_VALID && centre_ULTRA.y <= CENTRE_SENSOR_TOLERANCE && right_IR.x != NOT_VALID) {  // x  -  x //needs work at wall // now avoid corners brilliantly!
    	if((left_IR.y - right_IR.y) < 50 && 0 < centre_S_IR.x > -250 && centre_S_IR.x != NOT_VALID){//detects a side wall
			target.x = 200;
	  		target.y = -50;
	  		PRINT("Turning right   ");
		}
		else if((left_IR.y - right_IR.y) < 50 && centre_S_IR.x == NOT_VALID){
        	target.x = -200;
            target.y = -50;
            PRINT("Turning left   ");
		}
    	else {
    		target.x = (left_IR.x + right_IR.x)/2;
    		target.y = ROBOT_RADIUS;
    	}
    	PRINT("x - x         ");
  	}
  	else if (left_IR.x != NOT_VALID && centre_IR.y == NOT_VALID && right_IR.x == NOT_VALID) {  // -  x  x
    	target.x = -ROBOT_RADIUS;
    	target.y = 100;
    	PRINT("- x x         ");
  	}
  	else if (left_IR.x != NOT_VALID && centre_IR.y == NOT_VALID && right_IR.x == NOT_VALID) {  // -  -  x
    	target.x = random(-100,-200);
    	target.y = ROBOT_RADIUS;
    	PRINT("- - x         ");
  	}
  	else if (centre_ULTRA.y != NOT_VALID && centre_ULTRA.y < 100) { // checks for a wall close before the next one
  		DC.drive(-35, 0);
    	PRINT("0 x 0   just ultra      ");
    	delay(1000);
  	}
  	else if (left_IR.x == NOT_VALID && centre_ULTRA.y != NOT_VALID && centre_ULTRA.y < 500 && right_IR.x == NOT_VALID) {  // -  x  - 
    	target.x = 0;
    	target.y = -50;
    	PRINT("- x -         ");
  	}
  	else if (left_IR.x == NOT_VALID && centre_IR.y == NOT_VALID && right_IR.x != NOT_VALID) {  // x  -  -
    	target.x = random(100,200);
    	target.y = ROBOT_RADIUS;
    	PRINT("x - -         ");
  	}
  	else if (left_IR.x == NOT_VALID && centre_IR.y == NOT_VALID && right_IR.x == NOT_VALID) {  // -  -  -
    	target.x = 0;
    	target.y = ROBOT_RADIUS;
    	PRINT("- - -         ");
  	}
  	else {
  		target.x = -50;
  		target.y = -300;
  		PRINT("NO STATE      ");
  	}
  	PRINT("\r");
	  
    
  	return target;
}



void secondary_tactic(void) {
    PRINTLN("Starting secondary tactic:");
    Timer3.setPeriod(SECONDARY_TACTIC_PERIOD);
    Timer3.attachInterrupt(secondary_tactic_ISR);
    
    uint8_t operation_state = SEARCHING;
    int32_t last_weight_time = 0;
    int32_t target_time = 0;
    uint32_t weight_timeout = 0;
    
    int8_t weight_detection_count = 0;
    //int searching = 0;
    int i = 0;
  
  	Weight_Detect_t weight_locations = {{-1, -1}, {-1, -1}};
  	uint8_t curr_base = home_base;
  
  	buffer_initialize(&stop_buffer_x, STOP_BUFFER_SIZE);
  	buffer_initialize(&stop_buffer_y, STOP_BUFFER_SIZE);

    CartVec cart_target = {0, 0};
    PolarVec polar_target = {0, 0};
    bool is_stuck = false; //not used atm
    uint16_t countdown = 0; //not used atm
    uint32_t stuck_millis = 0; // stuck time
    uint32_t last_millis = millis(); 
    uint32_t program_start = millis();
  
  	char serial_byte = '\0';
  	bool enable_drive = true;
  	//play_sound(cena_main);
  
  	SERVO_COLOUR = LED_WHITE;
  
  	while(OPERATION_MODE == SECONDARY_MODE) {
        if ((millis() - program_start) > 300000) {
            PRINTLN("That\'s time!");
            play_sound(cena_main);
        }
        
        weight_locations = weight_detect();
    
    	curr_base = colour_detect();
    	IMU.update();
    
    	/// Check how many weights I've got ///
    	if (is_full()) {
      		operation_state = RETURNING;
    	}
        
        /// Check if I'm in a base ///
        if (curr_base == home_base && home_base != NO_BASE) { // In home base
            raise_magnets(); //PRINTLN("in base");
            retract_magnets();
            // Something else?
            // reverse a little
            //cart_target.x =  0;
            cart_target = get_local_target() ;//cart_target.y = -ROBOT_RADIUS;  // get_local_target();
            operation_state = SEARCHING;
            //searching = 0;
        }
        else if (curr_base == NO_BASE) {  // In arena
            extend_magnets();
            switch (operation_state) { 
                case SEARCHING:
                	if (is_full()) {
      					operation_state = RETURNING;
      					break;
    				}
                    SERVO_COLOUR = LED_WHITE; //white doesnt show up :(
                    cart_target = get_local_target();
                    
                    if (DIP8_S3.is_active()) {
                        if (weight_locations.left != NO_WEIGHT || weight_locations.right != NO_WEIGHT) { // If I see a weight
                            weight_detection_count++;
                            if (weight_detection_count > 120) {
                                weight_detection_count = 120;
                                PRINT(weight_detection_count);
                            }
                            /*if (weight_detection_count >= 2) {
                                operation_state = COLLECTING;
                                last_weight_time = millis();
                            }*/
                        }
                        else {
                            weight_detection_count--;
                            if (weight_detection_count < 0) {
                                weight_detection_count = 0;
                            }
                        }
                    }
          			break;
                    
        		/*case COLLECTING:
        			if (is_full()) {
      					operation_state = RETURNING;
      					break;
    				}
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
                        if (IR_VAR1.is_active() && IR_VAR2.is_active()) {
                            cart_target.x -= 50;
                        }
                        else if (IR_VAR1.is_active() && IR_VAR2.is_active()) {
                            cart_target.x += 50;
                        }
                       
          	        }
                  	else if ((millis() - last_weight_time) > weight_timeout) {  // If lost weights
                    	//raise_magnets();
                    	weight_timeout = 0;
                    	operation_state = SEARCHING;
                    	weight_detection_count = 0;
                  	}
                    // only have the magenets down for max time
                    
                    
                    break;*/
                case RETURNING:
                    SERVO_COLOUR = LED_BLUE;
                    //raise_magnets();
                    cart_target = get_local_target();
                    if (!is_full()) {
                        operation_state = SEARCHING;
                        //searching = 0;
                    }
                    break;
            }
        }
        else {  // In opposition_base
        	if (is_full()) {
      			operation_state = RETURNING;
      			break;
    		}
            SERVO_COLOUR = LED_CYAN;
            raise_magnets(); //PRINTLN("other base");
            cart_target = get_local_target();
        }
        
        /// Raise Magnets two seconds after losing weight
        if ((operation_state != COLLECTING) && (millis() - last_weight_time) > WEIGHT_RAISE_TIMEOUT) {  // If lost weight
            raise_magnets();
        }
    
    	/// Check for stuck ///
    	if ((millis() - stuck_millis) > 5000) { //flag reset after stuck
    		stuck_flag = false;
    	}
        
        if ((millis() - last_millis) > 80 && enable_drive == true){// && DIP8_S7.is_active()) { // i dont know if this if statement will work?
            buffer_store(&stop_buffer_x, cart_target.x);
            buffer_store(&stop_buffer_y, cart_target.y);
      
          	if (abs(stop_buffer_x.data[STOP_BUFFER_SIZE-1] - buffer_average(stop_buffer_x)) < 50 &&
              	abs(stop_buffer_y.data[STOP_BUFFER_SIZE-1] - buffer_average(stop_buffer_y)) < 50 &&
              	stuck_flag == false){
              	PRINTLN("stuck!");
              	DC.drive(-35, 0);
              	delay(1000);
              	DC.drive(0, -20);
              	delay(1000);
              	stuck_flag = true;
              	stuck_millis = millis();
              	for (i = 0; i <= STOP_BUFFER_SIZE; i++){
                  	buffer_store(&stop_buffer_x, 10000);
          		  	buffer_store(&stop_buffer_y, 10000);
              	}
            
          	// need to reset buffer as now it just drives backwards
          	// This only happens when in corner ie not very often
          	}
          	last_millis = millis();
    	}
    
    /// Perform tasks ///
    polar_target = cart_target.polar();
    if (operation_state == COLLECTING && colour_detect() == NO_BASE) {
        polar_target.r = polar_target.r/3;
    }
    point_towards_target(polar_target);
    if (enable_drive == false || !DIP8_S7.is_active()) {
        polar_target.r = 0;
        polar_target.theta = 0;
    }
    move_towards_target(polar_target);
    
    //PRINT(weight_timeout);
    //PRINT("                   \r");
    
    if (DIP8_S1.is_active()) {
      OPERATION_MODE = SECONDARY_MODE;
    }
    else {
      OPERATION_MODE = MANUAL_MODE;
    }
    
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
    
    if (DIP8_S1.is_active()) {
      OPERATION_MODE = PRIMARY_MODE;
    }
    else {
      OPERATION_MODE = MANUAL_MODE;
    }
    
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
        PRINTLN("\tExtending magnets");
        extend_magnets();
      }
      else if (serial_byte == RETRACT) {
        PRINTLN("\tRetracting magnets");
        retract_magnets();
      }
      else if (serial_byte == TOGGLE) {
        PRINTLN("\tToggling magnets");
        toggle_magnets();
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
    
    debug_sensors();
    
    //PRINTLN(LEFT_ROTATION*0.104719755);
    //PRINTLN(RIGHT_ROTATION*0.104719755);
    //PRINTLN();
    COLOUR.update();
    IMU.update();
    //PRINT(IR_VAR1.is_active()); PRINT(IR_VAR2.is_active()); PRINT(IR_VAR3.is_active()); PRINT('\r');
    /*PRINT(DIP8_S1.is_active());
    PRINT(DIP8_S2.is_active());
    PRINT(DIP8_S3.is_active());
    PRINT(DIP8_S4.is_active());
    PRINT(DIP8_S5.is_active());
    PRINT(DIP8_S6.is_active());
    PRINT(DIP8_S7.is_active());
    PRINT(DIP8_S8.is_active());
    PRINT('\r');*/
    //print_buffer(IMU.read(), IMU_BUFFER_SIZE); PRINT('\r');
    //COLOUR.read();
    //delay(200);
    
  }
  PRINTLN("\nReceiving control\n");
  Timer3.detachInterrupt();
}
