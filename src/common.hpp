#ifndef COMMON_H_
#define COMMON_H_

const int INFO = 10;
const int DBG = 5;
const int CRITICAL = 1;
extern const int LOG_LEVEL;

#define LOG(w, x)                                                    \
  {                                                                  \
    if (w <= LOG_LEVEL)                                              \
    {                                                                \
      cout << __FUNCTION__ << "::" << __LINE__ << "->" << x << endl; \
    }                                                                \
  }

const long NUM_PTS = 50;

const double LANE_SIZE = 4.0;
const long NUM_LANES = 3;
const double LANE_EPSILON = 0.1;
const double DT = 0.02;

const double MAX_SPEED = 49.5;
const double MAX_DEL_SPEED = 0.4; // 2 * .224?

#endif
