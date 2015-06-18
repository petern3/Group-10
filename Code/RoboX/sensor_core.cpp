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
Sensor_t IR_SHT1 = {IR_SHT1_PIN, IR_SHT_RANGE, -1};
Sensor_t IR_SHT2 = {IR_SHT2_PIN, IR_SHT_RANGE, -1};
Sensor_t IR_MED1 = {IR_MED1_PIN, IR_MED_RANGE, -1};
Sensor_t IR_MED2 = {IR_MED2_PIN, IR_MED_RANGE, -1};
Sensor_t IR_LNG1 = {IR_LNG1_PIN, IR_LNG_RANGE, -1};
Sensor_t IR_LNG2 = {IR_LNG2_PIN, IR_LNG_RANGE, -1};
Sensor_t USONIC1 = {USONIC1_PIN, ULTRASONIC, -1};
Sensor_t USONIC2 = {USONIC2_PIN, ULTRASONIC, -1};

Sensor_t IR_VAR1 = {IR_VAR1_PIN, IR_VAR, -1};
Sensor_t IR_VAR2 = {IR_VAR2_PIN, IR_VAR, -1};
Sensor_t IR_VAR3 = {IR_VAR3_PIN, IR_VAR, -1};


/// FUNCTIONS ///
void init_sensor_core(void) {
  Serial.print("Initializing sensors...");
  analogReference(INTERNAL2V56);
  Serial.println("done");
}


static void read_ir_sht(Sensor_t* to_read) {
  
  int16_t sensor_actual = -1;
  int16_t sensor_analog = analogRead(to_read->sensor_port);
  
  if (sensor_analog >= IR_SHT_MIN_ADC || sensor_analog <= IR_SHT_MAX_ADC) {
    sensor_actual = map(sensor_analog, 1023, 0, IR_SHT_MIN_MM, IR_SHT_MAX_MM);
  }
  to_read->sensor_value = sensor_actual;
}

static void read_ir_med(Sensor_t* to_read) {
  
  int16_t sensor_actual = -1;
  int16_t sensor_analog = analogRead(to_read->sensor_port);
  
  if (sensor_analog >= IR_MED_MIN_ADC || sensor_analog <= IR_MED_MAX_ADC) {
    sensor_actual = map(sensor_analog, 1023, 0, IR_MED_MIN_MM, IR_MED_MAX_MM);
  }
  to_read->sensor_value = sensor_actual;
}


static void read_ir_lng(Sensor_t* to_read) {
  
  int16_t sensor_actual = -1;
  int16_t sensor_analog = analogRead(to_read->sensor_port);
  
  if (sensor_analog >= IR_LNG_MIN_ADC || sensor_analog <= IR_LNG_MAX_ADC) {
    sensor_actual = map(sensor_analog, 1023, 0, IR_LNG_MIN_MM, IR_LNG_MAX_MM);
  }
  to_read->sensor_value = sensor_actual;
}


static void read_ultrasonic(Sensor_t* to_read) {
  
  int16_t sensor_analog = -1;
  int16_t sensor_actual = sensor_analog;
  to_read->sensor_value = sensor_actual;
}


static void read_sonar(Sensor_t* to_read) {
  
  int16_t sensor_analog = analogRead(to_read->sensor_port);
  int16_t sensor_actual = map(sensor_analog, 0, 1023, 0, 1000); 
  //if (sensor_actual < IR_LNG_MIN || sensor_actual > IR_LNG_MAX) {
  //  sensor_actual = -1;
  //}
}


void sensor_distance(Sensor_t* to_read) {
  
  if (to_read->sensor_type == IR_SHT_RANGE) {
    read_ir_sht(to_read);
  }
  else if (to_read->sensor_type == IR_MED_RANGE) {
    read_ir_med(to_read);
  }
  else if (to_read->sensor_type == IR_LNG_RANGE) {
    read_ir_lng(to_read);
  }
  else if (to_read->sensor_type == ULTRASONIC) {
    read_ultrasonic(to_read);
  }
  else if (to_read->sensor_type == SONAR) {
    read_sonar(to_read);
  }
  
}


static void read_ir_var(Sensor_t* to_read) {
  
  bool sensor_actual = digitalRead(to_read->sensor_port);
  to_read->sensor_value = sensor_actual;
}


void sensor_detect(Sensor_t* to_read) {
  
  if (to_read->sensor_type == IR_VAR) {
    read_ir_var(to_read);
  }
  
}


void read_sensors(void) {
  sensor_distance(&IR_SHT1);
  sensor_distance(&IR_SHT2);
  sensor_distance(&IR_MED1);
  sensor_distance(&IR_MED2);
  sensor_distance(&IR_LNG1);
  sensor_distance(&IR_LNG2);
  sensor_distance(&USONIC1);
  sensor_distance(&USONIC2);
  
  sensor_detect(&IR_VAR1);
  sensor_detect(&IR_VAR2);
  sensor_detect(&IR_VAR3);
  
}


