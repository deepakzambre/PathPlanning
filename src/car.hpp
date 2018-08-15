#ifndef CAR_H_
#define CAR_H_

#include <iostream>
#include <vector>
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

enum CarState
{
  Start,
  Cruise,
  LaneChange
};

class Car
{
public:
  double x = 0;
  double y = 0;
  double s = 0;
  double d = 6.0;
  double yaw = 0;
  double speed = 0;
  int lane = 1;

  CarState state = CarState::Start;

  double last_speed = 0.0;
  double last_lane = 1.0;
  double iteration_speed = 0.224;
  double target_lane = 1.0;
  double cruise_speed = 49;

  // Previous path data given to the Planner
  json old_path_x;
  json old_path_y;

  // Previous path's end s and d values
  double old_path_end_s = 0;
  double old_path_end_d = 0;

  vector<double> next_x;
  vector<double> next_y;

  void UpdateIterationSpeed();
  void EvaluateAndUpdateState(json &sensor_fusion);
  void CalculateTrajectory(
      vector<double> map_waypoints_x,
      vector<double> map_waypoints_y,
      vector<double> map_waypoints_s);
  void Set(double x, double y, double s, double d, double yaw, double speed);
  void Log();
};

#endif
