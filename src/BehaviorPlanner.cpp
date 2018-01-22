/*
 * BehaviorPlanner.cpp
 *
 *  Created on: Jan 21, 2018
 *      Author: joewei
 */
#include <iostream>
#include <algorithm>

#include "BehaviorPlanner.hpp"

#define BUFFER_GAP  20

FSM BehaviorPlanner::nextState(const Vehicle& ego, std::vector<Vehicle> predictions)
{
  for (int i = 0; i < 3; i++) {
    std::cout << "lane #" << i << " speed: " << laneSpeed(ego, predictions, i) << " m/s" << std::endl;
  }

  std::vector<FSM> states = successorStates();
  std::vector<float> costs;
  for (FSM state : states) {
    float cost = 0.0;

    calculateLaneChangeIntent(ego, state);
    cost += inefficiencyCost(ego, predictions, state);
    cost += insufficientGapCost(ego, predictions, state);
    std::cout << toString(state) << ": total cost=" << cost << std::endl;

    costs.push_back(cost);
  }

  int best_idx = distance(costs.begin(), min_element(costs.begin(), costs.end()));
  std::cout << "best index=" << best_idx << std::endl;

  this->state = states[best_idx];
  calculateLaneChangeIntent(ego, this->state);

  return this->state;
}


double BehaviorPlanner::laneSpeed(const Vehicle& ego, const std::vector<Vehicle>& predictions, int lane)
{
  std::vector<double> speeds;

  for (auto v : predictions) {
    if (v.lane == lane) {
      /* We only care the vehicles ahead of the ego vehicle and is in view */
      if (v.s > ego.s && (v.s - ego.s) < 60)
        speeds.push_back(v.speed);
    }
  }


  double speed = -1.0;
  if (speeds.size() > 0)
    speed = *std::max_element(speeds.begin(), speeds.end());

  return speed;
}


std::vector<FSM> BehaviorPlanner::successorStates()
{
  std::vector<FSM> states;

  switch (this->state) {
  case FSM::laneKeep:
    states.push_back(FSM::laneKeep);
    states.push_back(FSM::prepareLaneChangeLeft);
    states.push_back(FSM::prepareLaneChangeRight);
    break;

  case FSM::prepareLaneChangeLeft:
    states.push_back(FSM::laneKeep);
    states.push_back(FSM::prepareLaneChangeLeft);
    states.push_back(FSM::laneChangeLeft);
    break;

  case FSM::prepareLaneChangeRight:
    states.push_back(FSM::laneKeep);
    states.push_back(FSM::prepareLaneChangeRight);
    states.push_back(FSM::laneChangeRight);
    break;

  case FSM::laneChangeLeft:
    states.push_back(FSM::laneKeep);
    states.push_back(FSM::laneChangeLeft);
    break;

  case FSM::laneChangeRight:
    states.push_back(FSM::laneKeep);
    states.push_back(FSM::laneChangeRight);
    break;
  }

  return states;
}


void BehaviorPlanner::calculateLaneChangeIntent(const Vehicle& ego, const FSM state)
{
  intendedLane = ego.lane;
  finalLane = ego.lane;

  if (state == FSM::prepareLaneChangeLeft) {
    intendedLane = ego.lane - 1;
  } else if (state == FSM::prepareLaneChangeRight) {
    intendedLane = ego.lane + 1;
  } else if (state == FSM::laneChangeLeft) {
    intendedLane = ego.lane - 1;
    finalLane = ego.lane - 1;
  } else if (state == FSM::laneChangeRight) {
    intendedLane = ego.lane + 1;
    finalLane = ego.lane + 1;
  }

  this->targetLane = finalLane;
}


float BehaviorPlanner::inefficiencyCost(const Vehicle& ego, const std::vector<Vehicle>& predictions, const FSM state)
{
  /* Penalize the off road drive and wrong way driving */
  if (intendedLane > 2 || intendedLane < 0)
    return 1.0;

  if (finalLane > 2 || finalLane < 0)
    return 1.0;

  double proposed_speed_intended = laneSpeed(ego, predictions, intendedLane);
  //If no vehicle is in the proposed lane, we can travel at target speed.
  if (proposed_speed_intended < 0)
    proposed_speed_intended = this->targetSpeed;

  double proposed_speed_final = laneSpeed(ego, predictions, finalLane);
  if (proposed_speed_final < 0)
    proposed_speed_final = this->targetSpeed;

  float cost = (2.0 * this->targetSpeed - proposed_speed_intended - proposed_speed_final) / this->targetSpeed;

  return cost;
}


float BehaviorPlanner::insufficientGapCost(const Vehicle& ego, const std::vector<Vehicle>& predictions, const FSM state)
{
  float cost = 0.0;

  if (ego.lane == intendedLane)
    return 0.0;

  for (auto v : predictions) {
    /*
     * Don't change lane if there are other vehicles 20 meters ahead or behind
     * the ego vehicle
     */
    if (v.lane == intendedLane && abs(ego.s - v.s) <= BUFFER_GAP) {
      cost = 1.0;
      break;
    }
  }

  return cost;
}


std::string BehaviorPlanner::toString(FSM state)
{
  std::string s;

  switch (state) {
  case FSM::laneKeep:
    s = "KL";
    break;

  case FSM::prepareLaneChangeLeft:
    s = "PLCL";
    break;

  case FSM::prepareLaneChangeRight:
    s = "PLCR";
    break;

  case FSM::laneChangeLeft:
    s = "LCL";
    break;

  case FSM::laneChangeRight:
    s = "LCR";
    break;
  }

  return std::move(s);
}


