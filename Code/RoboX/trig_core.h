 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * trig_core.h
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-05-31
 * Edited:  2015-05-31
 * 
 * The header file of the Trig Core.
 * 
 *//////////////////////////////////////////////////////////////////////
 

#ifndef TRIG_CORE_H
#define TRIG_CORE_H

////////////////
/// INCLUDES ///
////////////////
#include "Arduino.h"
#include "config.h"
#include <inttypes.h>
#include <stdlib.h>


///////////////
/// DEFINES ///
///////////////
#define degrees_to_radians(a) 0.01745329251*a
#define radians_to_degrees(a) 57.2957795131*a

#define tau  6.28318530718
#define pi   3.14159265359
#define pi_2 1.57079632679
#define pi_4 0.78539816339


///////////////
/// CLASSES ///
///////////////
class CartVec;
class PolarVec;

class CartVec {
  public:
    float x;
    float y;
    void operator=(CartVec obj);
    void operator=(float obj[2]);
    void operator=(int8_t obj[2]);
    void operator=(int16_t obj[2]);
    bool operator==(CartVec obj);
    bool operator==(float obj[2]);
    bool operator==(int8_t obj[2]);
    bool operator==(int16_t obj[2]);
    CartVec operator+(CartVec obj);
    CartVec operator+(PolarVec obj);
    
    CartVec cart(void);
    PolarVec polar(void);
    //void set(float x, float y);
};

class PolarVec {
  public:
    float r;
    float theta;
    void operator=(PolarVec obj);
    void operator=(float obj[2]);
    void operator=(int8_t obj[2]);
    void operator=(int16_t obj[2]);
    bool operator==(PolarVec obj);
    bool operator==(float obj[2]);
    bool operator==(int8_t obj[2]);
    bool operator==(int16_t obj[2]);
    CartVec operator+(PolarVec obj);
    CartVec operator+(CartVec obj);
    
    CartVec cart(void);
    PolarVec polar(void);
    //void set(float r, float theta);
};


/////////////////
/// FUNCTIONS ///
/////////////////
PolarVec cart_to_polar(CartVec cart);
CartVec polar_to_cart(PolarVec polar);

#endif
