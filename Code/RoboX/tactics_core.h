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
#define PRIMARY_MODE '0'
#define SECONDARY_MODE '1'
#define MANUAL_MODE '2'
#define DEFAULT_MODE SECONDARY_MODE  // can change to any of the previous modes

///////////////
/// GLOBALS ///
///////////////
extern uint8_t OPERATION_MODE;

///////////////
/// STRUCTS ///
///////////////
typedef struct {
  float x;
  float y;
} CartVec_t;

typedef struct {
  float r;
  float theta;
} PolarVec_t;

/////////////////
/// FUNCTIONS ///
/////////////////
void init_tactics_core(void);
void primary_tactic(void);
void secondary_tactic(void);
void manual_mode(void);


#endif

