 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * map_core.cpp
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-26
 * Edited:  2015-05-26
 * 
 * Blurb
 * 
 *//////////////////////////////////////////////////////////////////////


/// INCLUDES ///
#include "map_core.h"
#include "exception_core.h"
#include "miscellaneous_core.h"


/// GLOBALS ///
static File BOTMAP;


/// FUNCTIONS ///
void init_map_core(void) {
  print_("Initializing SD card...");

  if (!SD.begin(CHIP_SELECT_PIN)) {
    activate_exception(&SD_ERROR);
    println_("failed");
    //report_exception(&SD_ERROR);
    return;
  }
  println_("done");
  print_("Initializing map...");
  
  println_("done");
}


int8_t get_terrain(Position_t coord) {
  // Gets the terrain at a specific point
  int8_t terrain_to_get = EMPTY;
  char dir[DIR_BUFFER] = {'\0'};
  sprintf(dir, "%s/%*d,%*d", MAP_DIR, 3, coord.x, 3, coord.y);
  
  if (SD.exists(dir)) {
    BOTMAP = SD.open(dir, FILE_READ);
    if (BOTMAP) {
      terrain_to_get = BOTMAP.read();
      BOTMAP.close();
    }
  }
  
  return terrain_to_get;
}


int8_t set_terrain(Position_t coord, uint8_t terrain_to_set) {
  // Sets a single coordinate to a given terrain type
  int8_t set_terrain_success = 0;
  char dir[DIR_BUFFER] = {'\0'};
  sprintf(dir, "%s/%*d,%*d", MAP_DIR, 3, coord.x, 3, coord.y);
  
  if (SD.exists(dir)) {
    SD.remove(dir);
  }
  
  if (terrain_to_set == EMPTY) {
    BOTMAP = SD.open(dir, FILE_WRITE);
    if (BOTMAP) {
      print_("Writing to ");
      print_(dir);
      BOTMAP.print(terrain_to_set);
      BOTMAP.close();
      println_("...done.");
    } else {
      // if the file didn't open, print an error:
      //activate_exception(&SD_ERROR);
      println_("...error opening");
      set_terrain_success = -1;
    }
  }
  
  return set_terrain_success;
}


int8_t set_wall(Position_t coord) {
  
  int8_t set_wall_success = 0;
  
  return set_wall_success;
}


int8_t increment_terrain(Position_t coord, int8_t increment) {
  // Increments a single coordinate up or down
  int8_t increment_terrain_success = 0;
  uint8_t prev_value = 0;
  char dir[DIR_BUFFER] = {'\0'};
  sprintf(dir, "%s/%*d,%*d", MAP_DIR, 3, coord.x, 3, coord.y);
  
  if (SD.exists(dir)) {
    BOTMAP = SD.open(dir, FILE_READ);
    if (BOTMAP) {
      prev_value = BOTMAP.read();
      BOTMAP.close();
    }
  } else {
    prev_value = 0;
  }
  
  increment_terrain_success = set_terrain(coord, (max(prev_value + increment, 0)));
  
  return increment_terrain_success;
}


void display_map(void) {
  int16_t index_x = 0;
  int16_t index_y = 0;
  Position_t curr = {0, 0};
  int8_t terrain_to_print = 0;
  char num[3] = {"\0"};
  /*
  for (index_y = -1; index_y <= NUM_NODES_Y; index_y++) {
    for (index_x = -1; index_x <= NUM_NODES_X; index_x++) {
      curr.x = index_x;
      curr.y = index_y;
      terrain_to_print = get_terrain(curr);
      sprintf(num, "%2d", terrain_to_print);
      
      #ifdef DEBUG_MAP_NUMBERS
        print_(num);
      #else
        if (terrain_to_print == -1) {
          print_(NODE_EDGE);
        } else if (terrain_to_print == 0) {
          print_(NODE_OPEN);
        } else if (terrain_to_print == 1) {
          print_(NODE_WALL);
        } else if (terrain_to_print == 2) {
          print_(NODE_BASE);
        } else if (terrain_to_print == 3) {
          print_(NODE_PACK);
        } else {
          print_(num);
        }
      #endif
      }
    println_();
  }*/
}

