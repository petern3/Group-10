 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * interrupt_core.cpp
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-07-17
 * Edited:  2015-07-17
 * 
 * Blurb
 * 
 *//////////////////////////////////////////////////////////////////////
 
////////////////
/// INCLUDES ///
////////////////
#include "interrupt_core.h"
#include "actuator_core.h"
#include "misc_core.h"
#include "sensor_core.h"

///////////////
/// GLOBALS ///
///////////////
static float ENCODER_INCREMENT = (TAU / ENCODER_PPR);
//static float ENCODER_MAX = (ENCODER_WRAP * TAU);
static uint32_t left_prev_time = 0;
static uint32_t right_prev_time = 0;

bool LEFT_DIR = DC_FORWARD;
bool RIGHT_DIR = DC_FORWARD;

volatile float LEFT_ROTATION = 0;
volatile float RIGHT_ROTATION = 0;

//////////////////
/// INTERRUPTS ///
//////////////////
void left_encoder_ISR(void) {
  
  uint32_t left_curr_time = micros();

  if (LEFT_DIR == DC_FORWARD) {
    LEFT_ROTATION = -ENCODER_INCREMENT*1000000 / (left_curr_time - left_prev_time);
  } else {
    LEFT_ROTATION = ENCODER_INCREMENT*1000000 / (left_curr_time - left_prev_time);
  }
  left_prev_time = left_curr_time;
  
}

void right_encoder_ISR(void) {
  
  uint32_t right_curr_time = micros();
  if (RIGHT_DIR == DC_FORWARD) {
    RIGHT_ROTATION = -ENCODER_INCREMENT*1000000 / (right_curr_time - right_prev_time);
  } else {
    RIGHT_ROTATION = ENCODER_INCREMENT*1000000 / (right_curr_time - right_prev_time);
  }
  right_prev_time = right_curr_time;
}


void reset_encoders(void) {
  if ((micros() - left_prev_time) > ENCODER_RESET_TIMEOUT) {
    LEFT_ROTATION = 0;
  }
  if ((micros() - right_prev_time) > ENCODER_RESET_TIMEOUT) {
    RIGHT_ROTATION = 0;
  }
}


void primary_tactic_ISR(void) {
  reset_encoders();
  update_sensors();
  
}

void secondary_tactic_ISR(void) {
  reset_encoders();
  update_sensors();
  
}

void manual_control_ISR(void) {
  reset_encoders();
  update_sensors();
  //PRINTLN("Interrupt");
}


