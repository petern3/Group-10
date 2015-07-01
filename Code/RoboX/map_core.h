 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * map_core.h
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-26
 * Edited:  2015-05-31
 * 
 * The header file of the Map Core.
 * 
 *//////////////////////////////////////////////////////////////////////


#ifndef MAP_CORE_H
#define MAP_CORE_H


/// INCLUDES ///
#include "Arduino.h"
#include "config.h"
#include <inttypes.h>
#include <stdlib.h>
#include <SPI.h>
#include <SD.h>


/// DEFINES ///


/// STRUCTS ///
typedef struct {
  int16_t x;
  int16_t y;
} Position_t;


/// GLOBALS ///


/// FUNCTIONS ///
void init_map_core(void);
void set_wall(Position_t coord);
void display_map(void);


#endif
