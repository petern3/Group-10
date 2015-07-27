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

////////////////
/// INCLUDES ///
////////////////
#include "Arduino.h"
#include "config.h"
#include <inttypes.h>
#include <stdlib.h>

///////////////
/// CLASSES ///
///////////////
class Exception {
  private:
    char* descript;
    char* sound;
  public:
    bool is_active;
    void initialize(char* descript, char* sound);
    void activate(void);
    void deactivate(void);
    void report(void);
};

///////////////
/// GLOBALS ///
///////////////
extern Exception SD_ERROR;
extern Exception MAP_READ_ERROR;
extern Exception MAP_WRITE_ERROR;
extern Exception DC_MOTOR_ERROR;
extern Exception COLOUR_SENSOR_ERROR;

/////////////////
/// FUNCTIONS ///
/////////////////
void init_exception_core(void);


#endif
