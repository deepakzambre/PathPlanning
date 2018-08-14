#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helper.hpp"
#include "car.hpp"
#include "common.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

const int LOG_LEVEL = 11;

int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  Car ego;
  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &ego](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object

          // Main car's localization Data
          ego.Set(j[1]["x"]
          ,j[1]["y"]
          ,j[1]["s"]
          ,j[1]["d"]
          ,j[1]["yaw"]
          ,j[1]["speed"]);
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          auto end_path_s = j[1]["end_path_s"];
          auto end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          ego.Log();


          /***************************************************************************/
          int prev_size = previous_path_x.size();
/*
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }

           bool too_close = false;

          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            float d = sensor_fusion[i][6];
            if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane + 2))
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);
              double check_car_s = sensor_fusion[i][5];

              check_car_s += ((double)prev_size * 0.02 * check_speed);
              if ((check_car_s > car_s) && (check_car_s - car_s) < 30)
              {
                too_close = true;
              }
            }
          }

          if (too_close)
          {
            ref_v -= .224;
          }
          else
          {
            ref_v = min(49.5, ref_v + .124);
          }
 */
/*
          vector<double> ptsx;
          vector<double> ptsy;

           double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if (prev_size < 2)
          {
            LOG(INFO, "No Previous points");

            ptsx.push_back(car_x - cos(car_yaw));
            ptsy.push_back(car_y - sin(car_yaw));

            ptsx.push_back(car_x);
            ptsy.push_back(car_y);
          }
          else
          {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            ref_yaw = atan2(
              ref_y - double(previous_path_y[prev_size - 2]),
              ref_x - double(previous_path_x[prev_size - 2]));

            ptsx.push_back(previous_path_x[prev_size - 2]);
            ptsy.push_back(previous_path_y[prev_size - 2]);

            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y);
          }

          vector<double> wp0 = getXY(car_s + 20, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptsx.push_back(wp0[0]);
          ptsy.push_back(wp0[1]);

          vector<double> wp1 = getXY(car_s + 40, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptsx.push_back(wp1[0]);
          ptsy.push_back(wp1[1]);

          vector<double> wp2 = getXY(car_s + 60, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
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

            LOG(INFO, "ptsx = " + to_string(ptsx[i]) + ", ptsy = " + to_string(ptsy[i]));
          }

          tk::spline s;
          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 60;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);
          LOG(INFO, "target dist = " + to_string(target_dist));

          double N = target_dist * 2.24 / (0.02 * ref_v);
          double x_addon = 0;
          double x_inc = target_x / N;
          for (int i = 1; i <= 200 - previous_path_x.size(); i++, x_addon += x_inc)
          {
            double next_x = x_addon * cos(ref_yaw) - s(x_addon) * sin(ref_yaw) + ref_x;
            double next_y = x_addon * sin(ref_yaw) + s(x_addon) * cos(ref_yaw) + ref_y;
            LOG(INFO, "Next x = " + to_string(next_x) + ", Next y = " + to_string(next_y));

            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
          }
 */
          /***************************************************************************/

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          next_x_vals.clear();
          next_y_vals.clear();

          if (previous_path_x.size() > 2)
          {
            int closest_waypoint = 0;
            double dist = numeric_limits<double>::max();
            for (int i = 0; i < previous_path_x.size(); i++)
            {
              double d = distance(ego.x, ego.y, previous_path_x[i], previous_path_y[i]);
              if (d < dist)
              {
                d = dist;
                closest_waypoint = i;
              }
              else if (d > dist)
              {
                break;
              }
            }

            if (dist == 0.0)
            {
              closest_waypoint++;
            }

            closest_waypoint += NUM_SKIP_PTS;

            for (int i = closest_waypoint;
                 i < min(closest_waypoint + NUM_REUSE_PTS, (long)previous_path_x.size());
                 i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
          }

          // 0.0001 <= dist_inc <= 0.008
          double dist_inc = 0.008;

          double base_s = ego.s;
          if (next_x_vals.size() > 0)
          {
            double theta = ego.yaw;
            if (next_x_vals.size() > 1)
            {
              theta = atan2((next_y_vals.back() - next_y_vals[next_y_vals.size() - 2]),
                            (next_x_vals.back() - next_x_vals[next_x_vals.size() - 2]));
            }

            vector<double> f = getFrenet(next_x_vals.back(), next_y_vals.back(), theta, map_waypoints_x, map_waypoints_y);
            base_s = f[0];
          }

          for (int i = 0; i < 50; i++)
          {
            double next_s = base_s + (i + 1) * dist_inc;
            double next_d = 6;

            vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }


          vector<double> p_xy{ego.x, ego.y};
          for (int i = 0; i < 3; i++)
          {
            double dist = distance(next_x_vals[i], next_y_vals[i], p_xy[0], p_xy[1]);
            LOG(DBG, "i = " + to_string(i) + "(x, y) = (" + to_string(next_x_vals[i]) + ", " + to_string(next_y_vals[i]) + "), dist = " + to_string(dist));
            p_xy[0] = next_x_vals[i];
            p_xy[1] = next_y_vals[i];
          }

          if (LOG_LEVEL > INFO)
          {
//            getchar();
          }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
