/*
 * Predictor.hpp
 *
 *  Created on: Jan 21, 2018
 *      Author: joewei
 */

#ifndef SRC_PREDICTOR_HPP_
#define SRC_PREDICTOR_HPP_

#include <vector>
#include "Vehicle.hpp"

class Predictor {
public:
  std::vector<Vehicle> predict(std::vector<std::vector<double>> sensor_fusion, int prev_size);
};



#endif /* SRC_PREDICTOR_HPP_ */
