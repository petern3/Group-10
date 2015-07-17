 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * interrupt_core.h
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-07-17
 * Edited:  2015-07-17
 * 
 * The header file of the Interrupt Core.
 * 
 *//////////////////////////////////////////////////////////////////////
 

#ifndef INTERRUPT_CORE_H
#define INTERRUPT_CORE_H


/// INCLUDES ///
#include "Arduino.h"
#include "config.h"
#include <inttypes.h>
#include <stdlib.h>


/// GLOBALS ///
extern bool LEFT_DIR;
extern bool RIGHT_DIR;

extern volatile float LEFT_ROTATION;
extern volatile float RIGHT_ROTATION;

/// FUNCTIONS ///
void left_encoder_ISR(void);
void right_encoder_ISR(void);

void primary_tactic_ISR(void);
void secondary_tactic_ISR(void);
void manual_control_ISR(void);


#endif
