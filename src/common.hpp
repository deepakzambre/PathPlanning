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

const double MAX_S = 6945.55;
const long NUM_PTS = 50;
const long NUM_REUSE_PTS = 1;
const long NUM_SKIP_PTS = 0;

const double LANE_SIZE = 4.0;
const double DT = 0.02;

const double MAX_SPEED = 22.0;
const double MAX_ACCEL = 10.0;
const double MAX_JERK = 10.0;



#endif