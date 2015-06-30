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


/// GLOBALS ///
static File BOTMAP;
static Position_t ROBOT_POSITION;
static float ROBOT_ROTATION;

// Robot should start at a set 240 mm from edge in home base.
static int16_t X_START = -ROBOT_RADIUS;
static int16_t X_END = (MAP_SIZE_X / MAP_SPACING) - ROBOT_RADIUS;
static int16_t Y_START = -ROBOT_RADIUS;
static int16_t Y_END = (MAP_SIZE_Y / MAP_SPACING) - ROBOT_RADIUS;


/// FUNCTIONS ///
static void init_edges(void);

void init_map_core(void) {
  PRINT("\tSD card...");
  
  if (!SD.begin(CHIP_SELECT_PIN)) {
    activate_exception(&SD_ERROR);
    PRINTLN("failed");
    //report_exception(&SD_ERROR);
    return;
  }
  PRINTLN("done");
  PRINT("\tMap...");
  if (SD.exists(MAP_DIR)) {
    SD.rmdir(MAP_DIR);
  }
  init_edges();
  PRINTLN("done");
}


int8_t get_terrain(Position_t coord) {
  // Gets the terrain at a specific point
  int8_t terrain_to_get = EMPTY;
  char dir[DIR_BUFFER] = {'\0'};
  sprintf(dir, "%s/%*d,%*d", MAP_DIR, XY_DEC_MAX, coord.x, XY_DEC_MAX, coord.y);
  
  if (SD.exists(dir)) {
    BOTMAP = SD.open(dir, FILE_READ);
    if (BOTMAP) {
      PRINT("Reading ");
      PRINT(dir);
      terrain_to_get = BOTMAP.read();
      BOTMAP.close();
      PRINTLN("...done.");
    }
  }
  
  return terrain_to_get;
}


static void set_terrain(Position_t coord, uint8_t terrain_to_set) {
  // Sets a single coordinate to a given terrain type
  char dir[DIR_BUFFER] = {'\0'};
  sprintf(dir, "%s/%*d,%*d", MAP_DIR, XY_DEC_MAX, coord.x, XY_DEC_MAX, coord.y);
  
  if (SD.exists(dir)) {
    SD.remove(dir);
  }
  
  if (terrain_to_set == EMPTY) {
    BOTMAP = SD.open(dir, FILE_WRITE);
    if (BOTMAP) {
      PRINT("Writing to ");
      PRINT(dir);
      BOTMAP.print(terrain_to_set);
      BOTMAP.close();
      PRINTLN("...done.");
    } else {
      // if the file didn't open, PRINT an error:
      //activate_exception(&MAP_WRITE_ERROR);
      PRINTLN("...error opening");
    }
  }
}


static uint8_t set_hard_bit(uint8_t terrain, bool is_hard){
  // Sets the hard bit to 'is_hard'
  if (is_hard) {
    return (terrain | HARDNESS_BIT);
  } else {
    return (terrain & ~HARDNESS_BIT);
  }
}

static bool get_hard_bit(uint8_t terrain) {
  // Get the hard bit of a given terrain
  return (terrain >> 7);
}

static void set_hard_wall(Position_t coord, bool is_hard=true) {
  // Sets a single coordinate be either soft or hard
  set_terrain(coord, set_hard_bit(get_terrain(coord), is_hard));
}

static void increment_terrain(Position_t coord, int8_t increment, bool is_hard=false) {
  // Increments a single coordinate up or down if:
  //  - the set mode is 'hard' and the coordinate is hard
  //  - the set mode is 'soft' and the coordinate is soft
  uint8_t old_terrain = get_terrain(coord);
  
  if ((is_hard == false and get_hard_bit(old_terrain) == HARD_WALL)) {
  } else {
    int16_t new_terrain = (old_terrain & ~HARDNESS_BIT) + increment;
    // Cap the terrain to 0 and the the maximum wall number
    if (new_terrain < 0) {
      new_terrain = 0;
    }
    else if (new_terrain > WALL_MAX) {
      new_terrain = WALL_MAX;
    }
    new_terrain = set_hard_bit(new_terrain, get_hard_bit(old_terrain));
    set_terrain(coord, new_terrain);
  }
  
}


void set_wall(Position_t coord, bool set_true=true, bool set_edge=false) { //set_true=false means it is removing the wall
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
  
  This is calibrated for a robot width of 480mm!
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
  if (set_edge == true) {
    increment = WALL_MAX;
  }
  else if (set_true == false) {
    increment = -WALL_INCR;
  }
  increment_terrain(coord, increment, true);
  
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


static void init_edges(void) {
  Position_t curr = {0,0};
  
  // Set 'top' and 'bottom' edges
  for (int16_t x = X_START; x < X_END; x++) {
    curr.x = x;
    curr.y = Y_START;
    set_wall(curr, true, true);
    curr.y = Y_END;
    set_wall(curr, true, true);
  }
  // Set 'left' and 'right' edges
  for (int16_t y = Y_START; y < Y_END; y++) {
    curr.x = X_START;
    curr.y = y;
    set_wall(curr, true, true);
    curr.x = X_END;
    set_wall(curr, true, true);
  }
}


void display_map(void) {
  int16_t index_x = 0;
  int16_t index_y = 0;
  Position_t curr = {0, 0};
  int8_t terrain_to_PRINT = 0;
  char num[3] = {"\0"};
  /*
  for (index_y = -1; index_y <= NUM_NODES_Y; index_y++) {
    for (index_x = -1; index_x <= NUM_NODES_X; index_x++) {
      curr.x = index_x;
      curr.y = index_y;
      terrain_to_PRINT = get_terrain(curr);
      sPRINTf(num, "%2d", terrain_to_PRINT);
      
      #ifdef DEBUG_MAP_NUMBERS
        PRINT_(num);
      #else
        if (terrain_to_PRINT == -1) {
          PRINT_(NODE_EDGE);
        } else if (terrain_to_PRINT == 0) {
          PRINT_(NODE_OPEN);
        } else if (terrain_to_PRINT == 1) {
          PRINT_(NODE_WALL);
        } else if (terrain_to_PRINT == 2) {
          PRINT_(NODE_BASE);
        } else if (terrain_to_PRINT == 3) {
          PRINT_(NODE_PACK);
        } else {
          PRINT_(num);
        }
      #endif
      }
    PRINTLN();
  }*/
}

