

#include "Arduino.h"
#include "map_core.h"

static uint8_t mymap[NUM_NODES_Y][NUM_NODES_X];


void init_map_core(void) {
  //mymap = (uint8_t**)calloc(NUM_NODES_X, sizeof(uint8_t*));
  //for(uint8_t i=0; i<NUM_NODES_X; i++) {
  //  (mymap)[i]=(uint8_t*)calloc(NUM_NODES_X, sizeof(uint8_t));
  //}
}


int8_t get_terrain(Position_t coord) {
  
  int8_t terrain_to_get = -1;

  if (coord.x < NUM_NODES_X && coord.x >= 0 &&
      coord.y < NUM_NODES_Y && coord.y >= 0) {
    terrain_to_get = mymap[coord.y][coord.x];
  }
  return terrain_to_get;
}


int8_t set_terrain(Position_t coord, uint8_t terrain_to_set) {
  
  int8_t set_terrain_success = -1;
  
  if (coord.x < NUM_NODES_X && coord.x >= 0 &&
      coord.y < NUM_NODES_Y && coord.y >= 0) {

    mymap[coord.y][coord.x] = terrain_to_set;
    set_terrain_success = 0;
  }
  return set_terrain_success;
}
