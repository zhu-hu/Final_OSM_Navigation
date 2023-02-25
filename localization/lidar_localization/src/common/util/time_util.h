//
// Created by zhibo on 10/13/19.
//

#ifndef SRC_TIME_UTIL_H
#define SRC_TIME_UTIL_H

#include <ctime>

namespace localization {
    namespace util {

        double GetTimeInterval(clock_t start_time);

        double GetTimeInterval(clock_t start_time, clock_t end_time);

    } // namespace util
} // namespace localization

#endif //SRC_TIME_UTIL_H
