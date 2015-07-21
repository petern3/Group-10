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
 
////////////////
/// INCLUDES ///
////////////////
#include "exception_core.h"
#include "voice_core.h"
#include "misc_core.h"

///////////////
/// GLOBALS ///
///////////////
Exception SD_ERROR;
Exception MAP_READ_ERROR;
Exception MAP_WRITE_ERROR;
Exception DC_MOTOR_ERROR;
Exception COLOUR_SENSOR_ERROR;

/////////////////
/// FUNCTIONS ///
/////////////////
void init_exception_core(void) {
  PRINT("\tExceptions...");
  
  SD_ERROR.initialize("Could not find SD card\0" , NULL_ERROR_SOUND);
  MAP_READ_ERROR.initialize("Could not read map file\0" , NULL_ERROR_SOUND);
  MAP_WRITE_ERROR.initialize("Could not write to map\0" , NULL_ERROR_SOUND);
  DC_MOTOR_ERROR.initialize("I can't move!\0" , NULL_ERROR_SOUND);
  COLOUR_SENSOR_ERROR.initialize("Could not find colour sensor\0", NULL_ERROR_SOUND);
  
  PRINTLN("done");
}

/////////////////////////////////
/// EXCEPTION CLASS FUNCTIONS ///
/////////////////////////////////
void Exception::initialize(char* descript, char* sound) {
  
  this->active = 0;
  this->descript = (char*)malloc((string_length(descript) + 1) * sizeof(char));
  this->sound = (char*)malloc((string_length(sound) + 1) * sizeof(char));
  
}

void Exception::activate(void) {
  this->active = true;
}

void Exception::deactivate(void) {
  this->active = false;
}

void Exception::report(void) {
  
  if (SOUNDS_ON && this->sound != NULL_ERROR_SOUND) {
    play_sound(this->sound);
  }
  PRINTLN(this->descript);
}

bool Exception::is_active(void) {
  return this->active;
}


