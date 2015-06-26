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
static Position_t ROBOT_POSITION;
static float ROBOT_ROTATION;


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
  if (SD.exists(MAP_DIR)) {
    SD.rmdir(MAP_DIR);
  }
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


static void set_terrain(Position_t coord, uint8_t terrain_to_set) {
  // Sets a single coordinate to a given terrain type
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
      activate_exception(&MAP_WRITE_ERROR);
      println_("...error opening");
    }
  }
}


static void increment_terrain(Position_t coord, int8_t increment) {
  // Increments a single coordinate up or down
  uint8_t prev_value = EMPTY;
  char dir[DIR_BUFFER] = {'\0'};
  sprintf(dir, "%s/%*d,%*d", MAP_DIR, 3, coord.x, 3, coord.y);
  
  if (SD.exists(dir)) {
    BOTMAP = SD.open(dir, FILE_READ);
    if (BOTMAP) {
      prev_value = BOTMAP.read();
      BOTMAP.close();
    }
  }
  set_terrain(coord, (max(prev_value + increment, 0)));
}


void set_wall(Position_t coord, bool set_true) {
  /*
                    [][][]12[][][]
                [][]              [][]
            [][]                      [][]
          []                              []
        []                                  []
      []                                      []
      []                                      []
    []                                          []
    []                                          []
  []                                              []
  []                                              []
  []                                              []
  12                      []                      12
  []                                              []
  []                                              []
  []                                              []
    []                                          []
    []                                          []
      []                                      []
      []                                      []
        []                                  []
          []                              []
            [][]                      [][]
                [][]              [][]
                    [][][]12[][][]
  
  The middle one is set to a 'hard' wall
  The rest (outer ones) are set to a 'soft' wall
  */
  int8_t edge_fill[17][2] = {
    { 1,12},{ 2,12},{ 3,12},
                          { 4,11},{ 5,11},
                                        { 6,10},{ 7,10},
                                                      { 8, 9},
                                                            { 9, 8},
                                                                  {10, 7},
                                                                  {10, 6},
                                                                        {11, 5},
                                                                        {11, 4},
                                                                              {12, 3},
                                                                              {12, 2},
                                                                              {12, 1}};
  int8_t edge_array_length = 16;
  Position_t edge_position = {0, 0};
  int8_t increment = WALL_INCR;
  if (set_true == true) {
    increment_terrain(coord, increment);  // set_terrain to add in checking whether hard or soft wall. Have setting of hard/sof wall here
  } else {
    increment = -WALL_INCR;
    increment_terrain(coord, increment);
  }
  
  
  
  // Change each corner of the circle
  for (int i=0; i<edge_array_length; i++) {
    edge_position.x = coord.x - edge_fill[i][0];
    edge_position.y = coord.y - edge_fill[i][1];
    increment_terrain(edge_position, increment);
    edge_position.x = coord.x - edge_fill[i][0];
    edge_position.y = coord.y + edge_fill[i][1];
    increment_terrain(edge_position, increment);
    edge_position.x = coord.x + edge_fill[i][0];
    edge_position.y = coord.y - edge_fill[i][1];
    increment_terrain(edge_position, increment);
    edge_position.x = coord.x + edge_fill[i][0];
    edge_position.y = coord.y + edge_fill[i][1];
    increment_terrain(edge_position, increment);
  }
  // Change the four edges in line with the centre
  edge_position.x = coord.x - 12;
  edge_position.y = coord.y;
  increment_terrain(edge_position, increment);
  edge_position.x = coord.x + 12;
  edge_position.y = coord.y;
  increment_terrain(edge_position, increment);
  edge_position.x = coord.x;
  edge_position.y = coord.y - 12;
  increment_terrain(edge_position, increment);
  edge_position.x = coord.x;
  edge_position.y = coord.y + 12;
  increment_terrain(edge_position, increment);
  
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

