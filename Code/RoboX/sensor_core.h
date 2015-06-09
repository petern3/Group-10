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


/// STRUCTS ///
typedef struct {
  uint8_t sensor_port;
  uint8_t sensor_type;
} Sensor_t;

/// GLOBALS ///
extern Sensor_t IR_MED1;

/// FUNCTIONS ///
void init_sensor_core(void);
//Sensor_t init_sensor(uint8_t sensor_port, uint8_t sensor_type);
int16_t sensor_read(Sensor_t to_read);


#endif
