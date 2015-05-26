

#ifndef CONFIG_H
#define CONFIG_H

/// MAP CONFIG ///

#define MAP_SIZE_X 5000  //4900  // mm  Wider than is long
#define MAP_SIZE_Y 2500  //2400  // mm
#define MAP_SPACING 500  //20  // mm


#define NODE_EDGE "##"
#define NODE_OPEN "  "
#define NODE_WALL "[]"
#define NODE_BASE "~~"
#define NODE_PACK "::"


/// DISPLAY CONFIG ///

#define BAUD_RATE 9600

//#define DEBUG_MAP_NUMBERS  // define if you want to diplay numbers instead of symbols


#endif CONFIG_H
