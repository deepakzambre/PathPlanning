#ifndef CAR_H_
#define CAR_H_

#include <iostream>
#include <vector>

using namespace std;

class Car
{
  public:
          double x = 0;
          double y = 0;
          double s = 0;
          double d = 0;
          double yaw = 0;
          double speed = 0;

          // Previous path data given to the Planner
          vector<double> previous_path_x;
          vector<double> previous_path_y;

          // Previous path's end s and d values
          double end_path_s = 0;
          double end_path_d = 0;

  void Set(double x, double y, double s, double d, double yaw, double speed);
  void Log();
};

#endif
