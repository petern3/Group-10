 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * misc_core.cpp
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-06-25
 * Edited:  2015-06-25
 * 
 * Blurb
 * 
 *//////////////////////////////////////////////////////////////////////

////////////////
/// INCLUDES ///
////////////////
#include "misc_core.h"

///////////////
/// GLOBALS ///
///////////////
float TAU = (2*PI);

/////////////////
/// FUNCTIONS ///
/////////////////
void print_buffer(int16_t* to_print, uint16_t len) {
  for (uint16_t i=0; i < len; i++) {
    PRINT(to_print[i]);
    PRINT("  ");
  }
  PRINTLN();
}

uint8_t string_length(char* to_count) {
  uint8_t index = 0;
  char s = to_count[index];
  
  while (s != '\0') {
    s = to_count[index];
    index++;
  }
  return index;
}
