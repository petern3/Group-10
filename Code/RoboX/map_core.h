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
#define MAP_DIR_BASE "BOTMAP"  // Format is "BOTMAPn/x.y" with no file extention
#define DIR_BUFFER_SIZE 22

#define EMPTY 0x00
#define PACK  0x01     // Soft or hard
#define BASE  0x02     //      |
#define WALL_MIN 0x04  //      00000100
#define WALL_MAX 0x7F  //      01111111
#define WALL_INCR 1

#define HARDNESS_BIT 0x80
#define SOFT_WALL 0
#define HARD_WALL 1


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
