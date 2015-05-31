 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * voice_core.h
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-30
 * Edited:  2015-05-31
 * 
 * The header file of the Voice Core.
 * 
 *//////////////////////////////////////////////////////////////////////
 

#ifndef VOICE_CORE_H
#define VOICE_CORE_H


/// INCLUDES ///
#include "Arduino.h"
#include "config.h"
#include <inttypes.h>


/// STRUCTS ///


/// GLOBALS ///
extern bool SOUNDS_ON;


/// FUNCTIONS ///
void init_voice_core(void);
void play_sound(String file_name);


#endif

