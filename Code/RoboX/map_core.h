

#ifndef MAP_CORE_H
#define MAP_CORE_H

/// DEFINES ///
#include "config.h"
#include <inttypes.h>


/// STRUCTS ///
typedef struct {
  uint8_t x;
  uint8_t y;
} Position_t;


/// GLOBALS ///
uint8_t NUM_NODES_X = MAP_SIZE_X / MAP_SPACING;
uint8_t NUM_NODES_Y = MAP_SIZE_Y / MAP_SPACING;

/// FUNCTIONS ///
void init_map_core(void);
int8_t get_terrain(Position_t coord);
int8_t set_terrain(Position_t coord, uint8_t terrain_to_set);


#endif
