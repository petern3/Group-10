 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * voice_core.cpp
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-30
 * Edited:  2015-05-31
 * 
 * Blurb
 * 
 *//////////////////////////////////////////////////////////////////////
 

/// INCLUDES ///
#include "voice_core.h"


/// GLOBALS ///
extern bool SOUNDS_ON = false;


/// FUNCTIONS ///
void init_voice_core(void) {
  Serial.print("Initializing speakers...");
  #ifdef ENABLE_SOUNDS
    SOUNDS_ON = true;
  #else
    SOUNDS_ON = false;
  #endif
  Serial.println("done");
}


void play_sound(String file_name) {
  
}
