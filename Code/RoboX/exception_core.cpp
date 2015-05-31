 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * exception_core.cpp
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
#include "exception_core.h"
#include "voice_core.h"


/// GLOBALS ///
Exception_t SD_ERROR = init_exception("Could not find SD card", NULL_ERROR_SOUND);


/// FUNCTIONS ///
void init_exception_core(void) {
  
}


Exception_t init_exception(String descript, String sound) {
  Exception_t* to_init;
  
  to_init->active = (bool)calloc(1, sizeof(bool));
  to_init->active = false;
  
  to_init->descript = (char*)calloc(descript.length(), sizeof(char));
  to_init->descript = descript;
  
  to_init->sound = (char*)calloc(sound.length(), sizeof(char));
  to_init->sound = sound;
  
  return *to_init;
}


void activate_exception(Exception_t* to_activate) {
  to_activate->active = true;
}


void deactivate_exception(Exception_t* to_deactivate) {
  to_deactivate->active = false;
}


void report_exception(Exception_t to_report) {
  
  if (SOUNDS_ON && to_report.sound != NULL_ERROR_SOUND) {
    play_sound(to_report.sound);
  }
  Serial.println(to_report.descript);
}




