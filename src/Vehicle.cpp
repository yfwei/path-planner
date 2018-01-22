/*
 * Vehicle.cpp
 *
 *  Created on: Jan 21, 2018
 *      Author: joewei
 */
#include <math.h>

#include "Vehicle.hpp"

Vehicle::Vehicle(int id, double x, double y, double vx, double vy, double s, double d, double yaw)
{
  this->id = id;
  this->x = x;
  this->y = y;
  this->vx = vx;
  this->vy = vy;
  this->s = s;
  this->d = d;
  this->speed = sqrt(vx * vx + vy * vy);
  this->yaw = yaw;

  /*
   * There are 3 lanes on the right-hand side of the road. Each has length of
   * 4 meters.
   */
  this->lane = (int)(d / 4.0);
}
