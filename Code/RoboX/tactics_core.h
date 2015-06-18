 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * tactics_core.h
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-06-17
 * Edited:  2015-06-18
 * 
 * The header file of the Tactics Core.
 * 
 *//////////////////////////////////////////////////////////////////////
 
 
#ifndef TACTICS_CORE_H
#define TACTICS_CORE_H


/// INCLUDES ///
#include "Arduino.h"
#include "config.h"
#include <inttypes.h>


/// STRUCTS ///


/// GLOBALS ///
extern bool FORCE_SECONDARY;


/// FUNCTIONS ///
void init_tactics_core(void);
void primary_tactic(void);
void secondary_tactic(void);


#endif

