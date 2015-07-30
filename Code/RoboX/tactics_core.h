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

////////////////
/// INCLUDES ///
////////////////
#include "Arduino.h"
#include "config.h"
#include <inttypes.h>
#include <TimerThree.h>

///////////////
/// DEFINES ///
///////////////
#define IDLE_MODE '0'
#define PRIMARY_MODE '1'
#define SECONDARY_MODE '2'
#define MANUAL_MODE '3'
//#define DEFAULT_MODE IDLE_MODE  // can change to any of the previous modes

#define SEARCHING 0
#define COLLECTING 1
#define RETURNING 2

///////////////
/// GLOBALS ///
///////////////
extern uint8_t OPERATION_MODE;

/////////////////
/// FUNCTIONS ///
/////////////////
void init_tactics_core(void);
void idle_mode(void);
void primary_tactic(void);
void secondary_tactic(void);
void manual_mode(void);


#endif

