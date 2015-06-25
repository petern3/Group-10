 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * miscellaneous_core.h
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-31
 * Edited:  2015-05-31
 * 
 * The header file of the Miscellaneous Core.
 * 
 *//////////////////////////////////////////////////////////////////////
 

#ifndef SENSOR_CORE_H
#define SENSOR_CORE_H


/// INCLUDES ///
#include "Arduino.h"
#include "config.h"
#include <inttypes.h>
#include <stdlib.h>


/// GLOBALS ///
extern float TAU;

/// FUNCTIONS ///
void print_(String to_print);
void println_(String to_print);


#endif
