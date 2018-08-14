#include "car.hpp"
#include "common.hpp"
#include <iostream>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void Car::Set(double in_x, double in_y, double in_s, double in_d, double in_yaw, double in_speed)
{
  x = in_x;
  y = in_y;
  s = in_s;
  d = in_d;
  yaw = in_yaw;
  speed = in_speed;
}

void Car::Log()
{
  LOG(INFO, "car x = " + to_string(x) + ", car y = " + to_string(y));
  LOG(INFO, "car s = " + to_string(s) + ", car d = " + to_string(d));
  LOG(INFO, "car speed = " + to_string(speed) + ", car yaw = " + to_string(yaw));
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
