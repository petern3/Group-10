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
#define IR_VAR_RANGE 0
#define IR_SHT_RANGE 1
#define IR_MED_RANGE 2
#define IR_LNG_RANGE 3
#define ULTRASONIC 4
#define SONAR 5

/// STRUCTS ///
typedef struct {
  uint8_t sensor_port;
  uint8_t sensor_type;
} Sensor_t;

/// GLOBALS ///
extern Sensor_t IR_MED1;

/// FUNCTIONS ///
void init_sensor_core(void);
int16_t sensor_read(Sensor_t to_read);


#endif
