#ifndef COMMON_H_
#define COMMON_H_

const int INFO = 10;
const int DBG = 5;
const int CRITICAL = 1;
extern const int LOG_LEVEL;

#define LOG(w, x)                                                       \
  {                                                                     \
    if (w <= LOG_LEVEL)                                                 \
    {                                                                   \
      cout << __FUNCTION__ << "::" << __LINE__ << "->" << x << endl;    \
    }                                                                   \
  }

const long NUM_PTS = 50;

const double LANE_SIZE = 4.0;
const double DT = 0.02;

const double MAX_SPEED = 49.5 * 1609.34 / (3600 * 50); // meter per tick; tick = 0.02 sec
const double MAX_ACCEL = 10 * 1609.34 / (50 * 50); // meter per tick^2
const double MAX_JERK = 50 * 1609.34 / (50 * 50 * 50); // meter per tick^3



#endif