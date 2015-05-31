 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * config.h
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-26
 * Edited:  2015-05-31
 * 
 * Blurb
 * 
 *//////////////////////////////////////////////////////////////////////
 

#ifndef CONFIG_H
#define CONFIG_H


/// MAP CONFIG ///
#define MAP_SIZE_X 5000  //4900  // mm  Wider than is long
#define MAP_SIZE_Y 2500  //2400  // mm
#define MAP_SPACING 500  //20  // mm


/// DEBUG CONFIG ///
#define BAUD_RATE 9600
//#define ENABLE_SOUNDS  // define if you want to play sounds
//#define DEBUG_ERROR_SOUNDS  // define if you only want error sounds


/// DISPLAY CONFIG ///
//#define DEBUG_MAP_NUMBERS  // define if you want to diplay numbers instead of symbols

#ifndef DEBUG_MAP_NUMBERS
  #define NODE_EDGE "##"
  #define NODE_OPEN "  "
  #define NODE_WALL "[]"
  #define NODE_BASE "~~"
  #define NODE_PACK "::"
#endif


/// ERROR CONFIG ///
#define NULL_ERROR_SOUND "\0"


#endif
