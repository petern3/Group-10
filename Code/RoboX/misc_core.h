 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * misc_core.h
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-31
 * Edited:  2015-05-31
 * 
 * The header file of the Misc Core.
 * 
 *//////////////////////////////////////////////////////////////////////
 

#ifndef MISC_CORE_H
#define MISC_CORE_H

////////////////
/// INCLUDES ///
////////////////
#include "Arduino.h"
#include "config.h"
#include <inttypes.h>
#include <stdlib.h>

///////////////
/// DEFINES ///
///////////////
#define degrees_to_radians(a) 0.01745329251*a
#define radians_to_degrees(a) 57.2957795131*a

#define tau  6.28318530718
#define pi   3.14159265359
#define pi_2 1.57079632679
#define pi_4 0.78539816339

///////////////
/// GLOBALS ///
///////////////
//extern float TAU;

/////////////////
/// FUNCTIONS ///
/////////////////
void print_buffer(int16_t* to_print, uint16_t len);
uint8_t string_length(char* to_count);

#endif
