 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * sensor_core.cpp
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-31
 * Edited:  2015-05-31
 * 
 * Blurb
 * 
 *//////////////////////////////////////////////////////////////////////
 

/// INCLUDES ///
#include "sensor_core.h"


/// GLOBALS ///
Sensor_t IR_MED_1 = {IR_MED1, IR_MED_RANGE};


/// FUNCTIONS ///
void init_sensor_core(void) {
  
}


static int16_t read_ir_var(Sensor_t to_read) {
  
  int16_t sensor_analog = analogRead(to_read.sensor_port);
  int16_t sensor_actual = map(sensor_analog, 1023, 0, 0, 100); 
  if (sensor_actual < IR_VAR_MIN || sensor_actual > IR_VAR_MAX) {
    sensor_actual = -1;
  }
  return sensor_actual;
}


static int16_t read_ir_sht(Sensor_t to_read) {
  
  int16_t sensor_analog = analogRead(to_read.sensor_port);
  int16_t sensor_actual = map(sensor_analog, 1023, 0, 0, 100); 
  if (sensor_actual < IR_SHT_MIN || sensor_actual > IR_SHT_MAX) {
    sensor_actual = -1;
  }
  return sensor_actual;
}

static int16_t read_ir_med(Sensor_t to_read) {
  
  int16_t sensor_analog = analogRead(to_read.sensor_port);
  int16_t sensor_actual = map(sensor_analog, 1023, 0, 0, 100); 
  if (sensor_actual < IR_MED_MIN || sensor_actual > IR_MED_MAX) {
    sensor_actual = -1;
  }
  return sensor_actual;
}


static int16_t read_ir_lng(Sensor_t to_read) {
  
  int16_t sensor_analog = analogRead(to_read.sensor_port);
  int16_t sensor_actual = map(sensor_analog, 1023, 0, 0, 100); 
  if (sensor_actual < IR_LNG_MIN || sensor_actual > IR_LNG_MAX) {
    sensor_actual = -1;
  }
  return sensor_actual;
}


static int16_t read_ultrasonic(Sensor_t to_read) {
  
  int16_t sensor_analog = 0;
  int16_t sensor_actual = sensor_analog;
  return sensor_actual;
}


static int16_t read_sonar(Sensor_t to_read) {
  
  int16_t sensor_analog = analogRead(to_read.sensor_port);
  int16_t sensor_actual = map(sensor_analog, 0, 1023, 0, 1000); 
  //if (sensor_actual < IR_LNG_MIN || sensor_actual > IR_LNG_MAX) {
  //  sensor_actual = -1;
  //}
  return sensor_actual;
}


int16_t sensor_read(Sensor_t to_read) {
  int16_t sensor_actual = 0;
  
  if (to_read.sensor_type == IR_VAR_RANGE) {
    sensor_actual = read_ir_var(to_read);
  }
  else if (to_read.sensor_type == IR_SHT_RANGE) {
    sensor_actual = read_ir_sht(to_read);
  }
  else if (to_read.sensor_type == IR_MED_RANGE) {
    sensor_actual = read_ir_med(to_read);
  }
  else if (to_read.sensor_type == IR_LNG_RANGE) {
    sensor_actual = read_ir_lng(to_read);
  }
  else if (to_read.sensor_type == ULTRASONIC) {
    sensor_actual = read_ultrasonic(to_read);
  }
  else if (to_read.sensor_type == SONAR) {
    sensor_actual = read_sonar(to_read);
  }
  return sensor_actual;
}
