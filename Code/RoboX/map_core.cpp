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

////////////////
/// INCLUDES ///
////////////////
#include "map_core.h"
#include "exception_core.h"

///////////////
/// GLOBALS ///
///////////////
static File BOTMAP_ROOT;
static File BOTMAP;
static Position_t ROBOT_POSITION;   // (x, y) from it's start position
static float ROBOT_ROTATION;  // bearing from North

// Robot should start at a set 240 mm from edge in home base.
static int16_t X_START = -ROBOT_RADIUS / MAP_SPACING;
static int16_t X_END = (MAP_SIZE_X - ROBOT_RADIUS) / MAP_SPACING;
static int16_t Y_START = -ROBOT_RADIUS / MAP_SPACING;
static int16_t Y_END = (MAP_SIZE_Y - ROBOT_RADIUS) / MAP_SPACING;

static uint8_t MAP_NUMBER = 0;
static char MAP_DIR[DIR_BUFFER_SIZE] = {'\0'};

/////////////////
/// FUNCTIONS ///
/////////////////
static void init_edges(void);
static void remove_old_map(void);

void init_map_core(void) {
  PRINT("\tSD card...");
  
  if (!SD.begin(CHIP_SELECT_PIN)) {
    PRINTLN("failed");
    SD_ERROR.activate();
    //SD_ERROR.report();
    return;
  }
  PRINTLN("done");
  
  PRINT("\tMap...");
  #if DEFAULT_MODE == PRIMARY_MODE
    PRINTLN();
    
    sprintf(MAP_DIR, "%s%d", MAP_DIR_BASE, MAP_NUMBER);
    
    if (SD.exists(MAP_DIR)) {
      remove_old_map();
    } else {
      SD.mkdir(MAP_DIR);
    }
    init_edges();
    PRINTLN("\t...done");
  #else
    PRINTLN("aborted");
  #endif
}


int8_t get_terrain(Position_t coord) {
  // Gets the terrain at a specific point
  int8_t terrain_to_get = EMPTY;
  char local_dir[DIR_BUFFER_SIZE] = {'\0'};
  sprintf(local_dir, "%s/%d.%d", MAP_DIR, coord.x, coord.y);
  
  if (SD.exists(local_dir)) {
    BOTMAP = SD.open(local_dir, FILE_READ);
    //PRINT("\t\tReading    ");
    //PRINT(local_dir);
    if (BOTMAP) {
      terrain_to_get = BOTMAP.read();
      BOTMAP.close();
      //PRINTLN("...done");
    } else {
      MAP_READ_ERROR.activate();
      //PRINTLN("...error opening");
    }
  }
  
  return terrain_to_get;
}


static void set_terrain(Position_t coord, uint8_t terrain_to_set) {
  // Sets a single coordinate to a given terrain type
  char local_dir[DIR_BUFFER_SIZE] = {'\0'};
  sprintf(local_dir, "%s/%d.%d", MAP_DIR, coord.x, coord.y);
  
  if (SD.exists(local_dir)) {
    SD.remove(local_dir);
  }
  
  if (terrain_to_set != EMPTY) {
    BOTMAP = SD.open(local_dir, FILE_WRITE);
    PRINT("\t\tWriting to ");
    PRINT(local_dir);
    if (BOTMAP) {
      BOTMAP.print(char(terrain_to_set));
      BOTMAP.close();
      PRINTLN("...done");
    } else {
      // if the file didn't open, print an error:
      MAP_WRITE_ERROR.activate();
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
  if ((is_hard == false && get_hard_bit(old_terrain) == HARD_WALL)) {
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
  for (uint8_t i=0; i<=edge_array_length; i++) {
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
  
  // Set top and bottom hard edges
  for (int16_t x = X_START; x <= X_END; x++) {
    curr.x = x;
    curr.y = Y_START;
    set_terrain(curr, 0xFF); //set_hard_bit(WALL_MAX, HARD_WALL));
    curr.y = Y_END;
    set_terrain(curr, 0xFF); //set_hard_bit(WALL_MAX, HARD_WALL));
  }
  // Set left and right hard edges
  for (int16_t y = Y_START; y <= Y_END; y++) {
    curr.x = X_START;
    curr.y = y;
    set_terrain(curr, 0xFF); //set_hard_bit(WALL_MAX, HARD_WALL));
    curr.x = X_END;
    set_terrain(curr, 0xFF); //set_hard_bit(WALL_MAX, HARD_WALL));
  }
  // Set top and bottom soft edges
  for (int16_t x = 0; x <= X_END + X_START; x++) {
    curr.x = x;
    curr.y = 0;
    set_terrain(curr, 0x7F);
    curr.y = Y_END + Y_START;
    set_terrain(curr, 0x7F);
  }
  // Set left and right soft edges
  for (int16_t y = 0; y <= Y_END + Y_START; y++) {
    curr.x = 0;
    curr.y = y;
    set_terrain(curr, 0x7F);
    curr.x = X_END + X_START;
    set_terrain(curr, 0x7F);
  }
  curr.x = 0;
  curr.y = 0;
  set_terrain(curr, PACK);
}


void display_map(void) {
  int16_t index_x = 0;
  int16_t index_y = 0;
  Position_t curr = {0, 0};
  uint8_t terrain_to_print = 0;
  char num[4] = {"\0"};
  
  for (index_y = Y_START; index_y <= Y_END; index_y++) {
    for (index_x = X_START; index_x <= X_END; index_x++) {
      curr.x = index_x;
      curr.y = index_y;
      terrain_to_print = get_terrain(curr);
      sprintf(num, "%3d", terrain_to_print);
      
      #ifdef DEBUG_MAP_NUMBERS
        PRINT(num);
      #else
        if (terrain_to_print == EMPTY) {
          PRINT(NODE_OPEN);
        } else if (terrain_to_print == PACK) {
          PRINT(NODE_PACK);
        } else if (terrain_to_print == BASE) {
          PRINT(NODE_BASE);
        } else {
          PRINT(num);
        }
      #endif
      }
    PRINTLN();
  }
  PRINTLN();
}


static void remove_old_map(void) {
  
  char local_dir[DIR_BUFFER_SIZE] = {'\0'};
  BOTMAP_ROOT = SD.open(MAP_DIR);
  
  if (BOTMAP_ROOT) {
    BOTMAP = BOTMAP_ROOT.openNextFile();
    while(BOTMAP) {
      sprintf(local_dir, "%s/%s", MAP_DIR, BOTMAP.name());
      BOTMAP.close();
      PRINT("\t\tDeleting   ");
      PRINT(local_dir);
      SD.remove(local_dir);
      PRINTLN("...done");
      BOTMAP = BOTMAP_ROOT.openNextFile();
    }
    BOTMAP_ROOT.close();
  }
}


int16_t bearing_to_origin(void) {
  
  // get bearing of robot from origin
  
  
  
}
