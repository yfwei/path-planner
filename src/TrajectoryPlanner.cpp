/*
 * TrajectoryPlanner.cpp
 *
 *  Created on: Jan 21, 2018
 *      Author: joewei
 */
#include <iostream>

#include "spline.h"
#include "TrajectoryPlanner.hpp"


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// MPH to MPS (Meters Per Second)
inline double mph2mps(double x) { return x * 0.44704; }


using namespace std;


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}

TrajectoryPlanner::TrajectoryPlanner(
    const std::vector<double>& map_waypoints_x,
    const std::vector<double>& map_waypoints_y,
    const std::vector<double>& map_waypoints_s)
{
  this->map_waypoints_x = map_waypoints_x;
  this->map_waypoints_y = map_waypoints_y;
  this->map_waypoints_s = map_waypoints_s;
}

void TrajectoryPlanner::generate(int lane, const Vehicle& ego, const std::vector<Vehicle>& predictions,
    const std::vector<double>& previous_path_x,
    const std::vector<double>& previous_path_y,
    std::vector<double>& next_x_vals,
    std::vector<double>& next_y_vals)
{
  /*
   * Don't change lane if the vehicle speed does not pick up.
   */
  if (ego.speed < mph2mps(30.0)) {
    lane = ego.lane;
    cout << "Picking up the speed..." << endl;
  }

  bool too_close = false;

  for (auto v : predictions) {
    float d = v.d;
    /* Car is in my lane */
    if (lane == v.lane) {
      double object_s = v.s;

      if ((object_s > ego.s) && (object_s - ego.s) < 30) {
        too_close = true;
      }
    }
  }

  if (too_close)
    this->ref_vel -= .224;
  else if (this->ref_vel < 30)
    this->ref_vel += .224 * 1.5;
  else if (this->ref_vel < 49.5)
    this->ref_vel += .224;

  vector<double> ptsx;
  vector<double> ptsy;

  auto ref_x = ego.x;
  auto ref_y = ego.y;
  auto ref_yaw = deg2rad(ego.yaw);

  int prev_size = previous_path_x.size();
  if (prev_size < 2) {
    auto prev_car_x = ego.x - cos(ref_yaw);
    auto prev_car_y = ego.y - sin(ref_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(ego.x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(ego.y);
  } else {
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  vector<double> next_wp0 = getXY(ego.s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(ego.s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(ego.s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  /* To the vehicle coordinates */
  for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  tk::spline s;
  s.set_points(ptsx, ptsy);

  /* Keep the previous way points to have a smooth trajectory */
  for (int i = 0; i < previous_path_x.size(); i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = 30;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  double x_add_on = 0;

  for(int i = 1; i <= (50 - previous_path_x.size()); i++) {
    double N = (target_dist / (.02 * this->ref_vel / 2.24));
    double x_point = x_add_on + target_x / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // to global map coordinates
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}

