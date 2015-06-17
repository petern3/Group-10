 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * sensor_core.h
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-31
 * Edited:  2015-05-31
 * 
 * The header file of the Sensor Core.
 * 
 *//////////////////////////////////////////////////////////////////////
 

#ifndef SENSOR_CORE_H
#define SENSOR_CORE_H


/// INCLUDES ///
#include "Arduino.h"
#include "config.h"
#include <inttypes.h>
#include <stdlib.h>

/// DEFINES ///
#define IR_SHT_RANGE 0
#define IR_MED_RANGE 1
#define IR_LNG_RANGE 2
#define ULTRASONIC 3
#define SONAR 4

#define IR_VAR 5
#define LIMIT_SW 6

/// STRUCTS ///
typedef struct {
  uint8_t sensor_port;
  uint8_t sensor_type;
  int16_t sensor_value;
} Sensor_t;

/// GLOBALS ///
extern Sensor_t IR_SHT1;
extern Sensor_t IR_SHT2;
extern Sensor_t IR_MED1;
extern Sensor_t IR_MED2;
extern Sensor_t IR_LNG1;
extern Sensor_t IR_LNG2;
extern Sensor_t USONIC1;
extern Sensor_t USONIC2;

extern Sensor_t IR_VAR1;
extern Sensor_t IR_VAR2;
extern Sensor_t IR_VAR3;

/// FUNCTIONS ///
void init_sensor_core(void);
//void sensor_distance(Sensor_t* to_read);
//void sensor_detect(Sensor_t* to_read);
void read_sensors(void);


#endif
