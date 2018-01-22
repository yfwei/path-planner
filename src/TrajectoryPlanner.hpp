/*
 * TrajectoryPlanner.hpp
 *
 *  Created on: Jan 21, 2018
 *      Author: joewei
 */

#ifndef SRC_TRAJECTORYPLANNER_HPP_
#define SRC_TRAJECTORYPLANNER_HPP_

#include <vector>

#include "Vehicle.hpp"
#include "BehaviorPlanner.hpp"

class TrajectoryPlanner {
public:
  TrajectoryPlanner(
      const std::vector<double>& map_waypoints_x,
      const std::vector<double>& map_waypoints_y,
      const std::vector<double>& map_waypoints_s);

  void generate(int lane, const Vehicle& ego, const std::vector<Vehicle>& predictions,
      const std::vector<double>& previous_path_x,
      const std::vector<double>& previous_path_y,
      std::vector<double>& next_x_vals,
      std::vector<double>& next_y_vals);

private:
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;

  double ref_vel{0.0};
};

#endif /* SRC_TRAJECTORYPLANNER_HPP_ */
