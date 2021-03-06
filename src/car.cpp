#include "car.hpp"
#include "common.hpp"
#include <iostream>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include "json.hpp"

#include "spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "helper.hpp"
#include "common.hpp"

using namespace std;
using json = nlohmann::json;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void Car::UpdateIterationSpeed()
{
  if (this->last_speed > this->cruise_speed && this->iteration_speed > 1.0)
  {
    this->iteration_speed = this->last_speed - .224;
  }
  else if (this->last_speed < this->cruise_speed && this->iteration_speed <= (MAX_SPEED - 1.0))
  {
    this->iteration_speed = min(49.5, this->last_speed + .224);
  }
}

void Car::EvaluateAndUpdateState(json &sensor_fusion)
{
  if (this->d < 0)
  {
    this->state = CarState::LaneChange;
    this->target_lane = 0;
    this->last_speed = this->iteration_speed;
    this->last_lane = this->lane;
    return;
  }

  if (this->d > (NUM_LANES * LANE_SIZE))
  {
    this->state = CarState::LaneChange;
    this->target_lane = NUM_LANES - 1;
    this->last_speed = this->iteration_speed;
    this->last_lane = this->lane;
    return;
  }

  if (this->state == CarState::LaneChange)
  {
    double target_lane_center = (this->target_lane * LANE_SIZE) + (LANE_SIZE / 2.0);
    if (target_lane_center - LANE_EPSILON <= this->d && this->d <= target_lane_center + LANE_EPSILON)
    {
      this->state = CarState::Cruise;
      this->UpdateIterationSpeed();
      this->last_speed = this->iteration_speed;
      this->last_lane = this->lane;
      return;
    }

    this->state = CarState::LaneChange;
    this->UpdateIterationSpeed();
    this->last_speed = this->iteration_speed;
    this->last_lane = this->lane;
    return;
  }

  int prev_size = this->old_path_x.size();
  double end_s = prev_size > 0 ? this->old_path_end_s : this->s;

  bool too_close = false;
  vector<bool> is_obstruction{false, false, false};
  vector<double> ahead_speed{MAX_SPEED, MAX_SPEED, MAX_SPEED};
  vector<double> ahead_dist{numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max()};
  for (int i = 0; i < sensor_fusion.size(); i++)
  {
    float check_d = sensor_fusion[i][6];
    double check_vx = sensor_fusion[i][3];
    double check_vy = sensor_fusion[i][4];
    double check_speed = sqrt(check_vx * check_vx + check_vy * check_vy);
    double check_s = sensor_fusion[i][5];
    int check_lane = (int)check_d / 4;

    if ((check_s <= this->s && (this->s - check_s) <= 5) ||
        (this->s < check_s && (check_s - this->s) <= 20)) //abs(check_s - this->s) < 20)
    {
      is_obstruction[check_lane] = true;
    }

    if (check_s < ahead_dist[check_lane] && this->s < check_s)
    {
      if (check_s < (this->s + 20))
      {
        ahead_speed[check_lane] = 2.2369 * check_speed - 20;
      }
      else if (check_s < (this->s + 30))
      {
        ahead_speed[check_lane] = 2.2369 * check_speed - 10;
      }
      else if (check_s < (this->s + 50))
      {
        ahead_speed[check_lane] = 2.2369 * check_speed;
      }
    }

    if (check_d < (2 + 4 * this->lane + 2) && check_d > (2 + 4 * this->lane - 2))
    {
      //double future_check_s = check_s + ((double)prev_size * 0.02 * check_speed);
      //if (check_s > this->s && (future_check_s > end_s) && (future_check_s - end_s) < 30)
      if (check_s > this->s && (check_s - this->s) < 50)
      {
        too_close = true;
      }
    }
  }

  if (too_close)
  {
    if (this->lane == 0)
    {
      if (!is_obstruction[1])
      {
        if (ahead_speed[0] < ahead_speed[1])
        {
          this->state = CarState::LaneChange;
          this->target_lane = 1;
          this->cruise_speed = ahead_speed[1];
        }
        else
        {
          this->state = CarState::Cruise;
          this->cruise_speed = ahead_speed[0];
        }
      }
      else
      {
        this->state = CarState::Cruise;
        this->cruise_speed = ahead_speed[0];
      }
    }
    else if (this->lane == 2)
    {
      if (!is_obstruction[1])
      {
        if (ahead_speed[2] < ahead_speed[1])
        {
          this->state = CarState::LaneChange;
          this->target_lane = 1;
          this->cruise_speed = ahead_speed[1];
        }
        else
        {
          this->state = CarState::Cruise;
          this->cruise_speed = ahead_speed[2];
        }
      }
      else
      {
        this->state = CarState::Cruise;
        this->cruise_speed = ahead_speed[2];
      }
    }
    else
    {
      int next_lane = this->lane;
      if (!is_obstruction[this->lane - 1] && ahead_speed[this->lane - 1] > ahead_speed[this->lane] && ahead_speed[this->lane - 1] > ahead_speed[this->lane + 1])
      {
        this->state = CarState::LaneChange;
        this->target_lane = this->lane - 1;
        this->cruise_speed = ahead_speed[this->lane - 1];
      }
      else if (!is_obstruction[this->lane + 1] && ahead_speed[this->lane + 1] > ahead_speed[this->lane] && ahead_speed[this->lane + 1] > ahead_speed[this->lane - 1])
      {
        this->state = CarState::LaneChange;
        this->target_lane = this->lane + 1;
        this->cruise_speed = ahead_speed[this->lane + 1];
      }
      else
      {
        this->state = CarState::Cruise;
        this->cruise_speed = ahead_speed[this->lane];
      }
    }
  }
  else
  {
    this->state = CarState::Cruise;
    this->cruise_speed = ahead_speed[this->lane];
  }

  this->UpdateIterationSpeed();
  this->last_speed = this->iteration_speed;
  this->last_lane = this->lane;
  LOG(INFO, "max lane speeds = " + to_string(ahead_speed[0]) + "," + to_string(ahead_speed[1]) + "," + to_string(ahead_speed[2]))
  LOG(INFO, "Cruise speed = " + to_string(this->cruise_speed));
  LOG(INFO, "state = " + to_string(this->state));
}

void Car::CalculateTrajectory(
    vector<double> map_waypoints_x,
    vector<double> map_waypoints_y,
    vector<double> map_waypoints_s)
{
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = this->x;
  double ref_y = this->y;
  double ref_yaw = deg2rad(this->yaw);

  int prev_size = this->old_path_x.size();
  if (prev_size < 2)
  {
    LOG(INFO, "No Previous points");

    ptsx.push_back(this->x - cos(this->yaw));
    ptsy.push_back(this->y - sin(this->yaw));

    ptsx.push_back(this->x);
    ptsy.push_back(this->y);
  }
  else
  {
    ref_x = this->old_path_x[prev_size - 1];
    ref_y = this->old_path_y[prev_size - 1];
    ref_yaw = atan2(
        ref_y - double(this->old_path_y[prev_size - 2]),
        ref_x - double(this->old_path_x[prev_size - 2]));

    ptsx.push_back(this->old_path_x[prev_size - 2]);
    ptsy.push_back(this->old_path_y[prev_size - 2]);

    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y);
  }

  double end_s = prev_size > 0 ? this->old_path_end_s : this->s;
  vector<double> wp0 = getXY(end_s + 30, 2 + 4 * this->target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  ptsx.push_back(wp0[0]);
  ptsy.push_back(wp0[1]);

  vector<double> wp1 = getXY(end_s + 60, 2 + 4 * this->target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  ptsx.push_back(wp1[0]);
  ptsy.push_back(wp1[1]);

  vector<double> wp2 = getXY(end_s + 90, 2 + 4 * this->target_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  ptsx.push_back(wp2[0]);
  ptsy.push_back(wp2[1]);

  for (int i = 0; i < ptsx.size(); i++)
  {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x * cos(-1 * ref_yaw) - shift_y * sin(-1 * ref_yaw);
    ptsy[i] = shift_x * sin(-1 * ref_yaw) + shift_y * cos(-1 * ref_yaw);

    if (i != 0 && ptsx[i] <= ptsx[i - 1])
    {
      ptsx[i] = ptsx[i] + 1;
    }
  }

  tk::spline s;
  s.set_points(ptsx, ptsy);

  this->next_x.clear();
  this->next_y.clear();

  for (int i = 0; i < prev_size; i++)
  {
    this->next_x.push_back(this->old_path_x[i]);
    this->next_y.push_back(this->old_path_y[i]);
  }

  double target_x = 30;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  double N = target_dist * 2.24 / (0.02 * this->iteration_speed);
  double x_addon = 0;
  double x_inc = target_x / N;
  for (int i = 1; i <= 50 - prev_size; i++)
  {
    x_addon += x_inc;
    double next_x = x_addon * cos(ref_yaw) - s(x_addon) * sin(ref_yaw) + ref_x;
    double next_y = x_addon * sin(ref_yaw) + s(x_addon) * cos(ref_yaw) + ref_y;

    this->next_x.push_back(next_x);
    this->next_y.push_back(next_y);
  }
}

void Car::Set(double in_x, double in_y, double in_s, double in_d, double in_yaw, double in_speed)
{
  x = in_x;
  y = in_y;
  s = in_s;
  d = in_d;
  yaw = in_yaw;
  speed = in_speed;
  lane = (int)d / 4;
}

void Car::Log()
{
  LOG(INFO, "************************************");
  LOG(INFO, "car x = " + to_string(x) + ", car y = " + to_string(y) + ", car yaw = " + to_string(yaw));
  LOG(INFO, "car x = " + to_string(x) + ", car y = " + to_string(y) + ", car yaw = " + to_string(yaw));
  LOG(INFO, "car s = " + to_string(s) + ", car d = " + to_string(d));
  LOG(INFO, "car speed = " + to_string(speed) + ", car target speed = " + to_string(iteration_speed));
  LOG(INFO, "car lane = " + to_string(lane) + ", car target lane = " + to_string(target_lane));
}

vector<double> JMT(vector<double> start, vector<double> end, double T)
{
  /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */

  MatrixXd A = MatrixXd(3, 3);
  A << T * T * T, T * T * T * T, T * T * T * T * T,
      3 * T * T, 4 * T * T * T, 5 * T * T * T * T,
      6 * T, 12 * T * T, 20 * T * T * T;

  MatrixXd B = MatrixXd(3, 1);
  B << end[0] - (start[0] + start[1] * T + .5 * start[2] * T * T),
      end[1] - (start[1] + start[2] * T),
      end[2] - start[2];

  MatrixXd Ai = A.inverse();

  MatrixXd C = Ai * B;

  vector<double> result = {start[0], start[1], .5 * start[2]};
  for (int i = 0; i < C.size(); i++)
  {
    result.push_back(C.data()[i]);
  }

  return result;
}
