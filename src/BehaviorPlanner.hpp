/*
 * BehaviorPlanner.hpp
 *
 *  Created on: Jan 21, 2018
 *      Author: joewei
 */

#ifndef SRC_BEHAVIORPLANNER_HPP_
#define SRC_BEHAVIORPLANNER_HPP_

#include <vector>
#include <string>

#include "Vehicle.hpp"

/*
 * Finite State Machine
 */
enum class FSM {
  laneKeep,
  prepareLaneChangeLeft,
  prepareLaneChangeRight,
  laneChangeLeft,
  laneChangeRight
};

class BehaviorPlanner {
public:
  BehaviorPlanner(double targetSpeed) {
    this->targetSpeed = targetSpeed;
  };

  FSM nextState(const Vehicle& ego, std::vector<Vehicle> predictions);

  double targetLane{1};

private:
  /*
   * Lane speed in MPS (Meters Per Second)
   */
  double laneSpeed(const Vehicle& ego, const std::vector<Vehicle>& predictions, int lane);

  /*
   * Get next candidate states
   */
  std::vector<FSM> successorStates();

  /*
   * Make the vehicle drive in the fastest possible lane
   */
  float inefficiencyCost(const Vehicle& ego, const std::vector<Vehicle>& predictions, const FSM state);

  /*
   * Penalize the lane change without enough gap
   */
  float insufficientGapCost(const Vehicle& ego, const std::vector<Vehicle>& predictions, const FSM state);

  void calculateLaneChangeIntent(const Vehicle& ego, const FSM state);

  std::string toString(FSM state);

  FSM state {FSM::laneKeep};
  double targetSpeed;

  int intendedLane{1};
  int finalLane{1};
};



#endif /* SRC_BEHAVIORPLANNER_HPP_ */
