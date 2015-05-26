

//#define DEBUG_MAP_NUMBERS  // whether to print numbers for the map

#define MAP_SIZE_X 5000  //4900  // mm  Wider than is long
#define MAP_SIZE_Y 2500  //2400  // mm
#define MAP_SPACING 40  //20  // mm


#define NODE_EDGE "##"
#define NODE_OPEN "  "
#define NODE_WALL "[]"
#define NODE_BASE "~~"
#define NODE_PACK "::"


/// STRUCTS ///
typedef struct {
  uint8_t x;
  uint8_t y;
} Position_t;


/// GLOBALS ///
// How wide is the internal map in terrain slots?
static uint8_t NUM_NODES_X = MAP_SIZE_X / MAP_SPACING;
static uint8_t NUM_NODES_Y = MAP_SIZE_Y / MAP_SPACING;
