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
//#define NUM_NODES_X (MAP_SIZE_X / MAP_SPACING)
//#define NUM_NODES_Y (MAP_SIZE_Y / MAP_SPACING)


/// STRUCTS ///
typedef struct {
  int16_t x;
  int16_t y;
} Position_t;


/// GLOBALS ///


/// FUNCTIONS ///
void init_map_core(void);
//int8_t get_terrain(Position_t coord);
//int8_t set_terrain(Position_t coord, uint8_t terrain_to_set);
void set_wall(Position_t coord);
void display_map(void);


#endif
