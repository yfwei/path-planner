/*
 * Predictor.cpp
 *
 *  Created on: Jan 21, 2018
 *      Author: joewei
 */

#include <vector>
#include "Predictor.hpp"

std::vector<Vehicle> Predictor::predict(std::vector<std::vector<double>> sensor_fusion, int prev_size)
{
  std::vector<Vehicle> vehicles;

  for (auto vehicle : sensor_fusion) {
    auto v = Vehicle(vehicle[0], vehicle[1], vehicle[2], vehicle[3], vehicle[4],
        vehicle[5], vehicle[6]);

    /* Project the s value in the future */
    v.s += (double)prev_size * 0.02 * v.speed;

    vehicles.push_back(v);
  }

  return vehicles;
}
