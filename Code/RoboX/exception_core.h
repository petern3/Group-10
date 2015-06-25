 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * exception_core.h
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-30
 * Edited:  2015-05-31
 * 
 * The header file of the Exception Core.
 * 
 *//////////////////////////////////////////////////////////////////////
 

#ifndef EXCEPTION_CORE_H
#define EXCEPTION_CORE_H


/// INCLUDES ///
#include "Arduino.h"
#include "config.h"
#include <inttypes.h>
#include <stdlib.h>


/// STRUCTS ///
typedef struct {
  uint8_t active;
  String* descript;
  String* sound;
} Exception_t ;


/// GLOBALS ///
extern Exception_t SD_ERROR;


/// FUNCTIONS ///
void init_exception_core(void);
Exception_t init_exception(String descript, String sound);
void activate_exception(Exception_t* to_activate);
void deactivate_exception(Exception_t* to_deactivate);
void report_exception(Exception_t* to_report);


#endif
