/*
 * Vehicle.hpp
 *
 *  Created on: Jan 21, 2018
 *      Author: joewei
 */

#ifndef SRC_VEHICLE_HPP_
#define SRC_VEHICLE_HPP_

#include <ostream>

class Vehicle {
public:
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
  double yaw;
  double speed;
  int lane;

  Vehicle(int id, double x, double y, double vx, double vy, double s, double d, double yaw=0.0);

  bool operator<(const Vehicle &rhs) const { return s < rhs.s; }

};



#endif /* SRC_VEHICLE_HPP_ */
