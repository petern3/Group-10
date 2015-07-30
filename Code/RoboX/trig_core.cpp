 ///////////////////////////////////////////////////////////////////////
/*----------------------------------------------------------------------
 * trig_core.cpp
 *----------------------------------------------------------------------
 * 
 * Author:  Peter Nicholls
 * Created: 2015-06-25
 * Edited:  2015-06-25
 * 
 * Blurb
 * 
 *//////////////////////////////////////////////////////////////////////

////////////////
/// INCLUDES ///
////////////////
#include "trig_core.h"


/////////////////////////////////////////
/// CARTESIAN 2VECTOR CLASS FUNCTIONS ///
/////////////////////////////////////////
void CartVec::operator=(CartVec obj) {
  this->x = obj.x;
  this->y = obj.y;
}

void CartVec::operator=(float obj[2]) {
  this->x = obj[0];
  this->y = obj[1];
}

void CartVec::operator=(int8_t obj[2]) {
  this->x = obj[0];
  this->y = obj[1];
}

void CartVec::operator=(int16_t obj[2]) {
  this->x = obj[0];
  this->y = obj[1];
}

bool CartVec::operator==(CartVec obj) {
  if (this->x == obj.x && this->y == obj.y) {
    return true;
  }
  return false;
}

bool CartVec::operator==(float obj[2]) {
  if (this->x == obj[0] && this->y == obj[1]) {
    return true;
  }
  return false;
}

bool CartVec::operator==(int8_t obj[2]) {
  if (this->x == obj[0] && this->y == obj[1]) {
    return true;
  }
  return false;
}

bool CartVec::operator==(int16_t obj[2]) {
  if (this->x == obj[0] && this->y == obj[1]) {
    return true;
  }
  return false;
}

CartVec CartVec::operator+(CartVec obj) {
  CartVec sum = {this->x + obj.x, this->y + obj.y};
  return sum;
}

CartVec CartVec::operator+(PolarVec obj) {
  CartVec temp = obj.cart();
  CartVec sum = {this->x + temp.x, this->y + temp.y};
  return sum;
}

CartVec CartVec::cart(void) {
  return *this;
}

PolarVec CartVec::polar(void) {
  // Converts it to a bearing from -pi to pi where 0 is North
  PolarVec polar = {0, 0};
  if (this->x != 0 && this->y != 0) {
    polar.r = sqrt((this->x*this->x) + (this->y*this->y));
    if (this->x >= 0) {
      polar.theta = pi_2 - atan(this->y / this->x);
    } else {
      polar.theta = -pi_2 - atan(this->y / this->x);
    }
  }
  return polar;
}
/*
void CartVec::set(float set_x, float set_y) {
  this->x = set_x;
  this->y = set_y;
}
*/

/////////////////////////////////////
/// POLAR 2VECTOR CLASS FUNCTIONS ///
/////////////////////////////////////
void PolarVec::operator=(PolarVec obj) {
  this->r = obj.r;
  this->theta = obj.theta;
}

void PolarVec::operator=(float obj[2]) {
  this->r = obj[0];
  this->theta = obj[1];
}

void PolarVec::operator=(int8_t obj[2]) {
  this->r = obj[0];
  this->theta = obj[1];
}

void PolarVec::operator=(int16_t obj[2]) {
  this->r = obj[0];
  this->theta = obj[1];
}

bool PolarVec::operator==(PolarVec obj) {
  if (this->r == obj.r && this->theta == obj.theta) {
    return true;
  }
  return false;
}

bool PolarVec::operator==(float obj[2]) {
  if (this->r == obj[0] && this->theta == obj[1]) {
    return true;
  }
  return false;
}

bool PolarVec::operator==(int8_t obj[2]) {
  if (this->r == obj[0] && this->theta == obj[1]) {
    return true;
  }
  return false;
}

bool PolarVec::operator==(int16_t obj[2]) {
  if (this->r == obj[0] && this->theta == obj[1]) {
    return true;
  }
  return false;
}

CartVec PolarVec::operator+(PolarVec obj) {
  CartVec temp1 = this->cart();
  CartVec temp2 = obj.cart();
  
  return temp1 + temp2;
}

CartVec PolarVec::operator+(CartVec obj) {
  CartVec temp = this->cart();
  CartVec sum = {temp.x + obj.x, temp.y + obj.y};
  return sum;
}

CartVec PolarVec::cart(void) {
  // Converts it from a bearing (from -pi to pi where 0 is North) to cartesian
  CartVec cart = {0, 0};
  cart.x = this->r * cos(this->theta + 90);
  cart.y = this->r * sin(this->theta + 90);
  
  return cart;
}

PolarVec PolarVec::polar(void) {
  return *this;
}
/*
void PolarVec::set(float set_r, float set_theta) {
  this->r = set_r;
  this->theta = set_theta;
}
*/

PolarVec cart_to_polar(CartVec cart) {
  // Converts it to a bearing from -pi to pi where 0 is North
  return cart.polar();
}

CartVec polar_to_cart(PolarVec polar) {
  // Converts it from a bearing (from -pi to pi where 0 is North) to cartesian
  return polar.cart();
}


