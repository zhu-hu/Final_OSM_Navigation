//
// Created by zhibo on 10/13/19.
//

#include "time_util.h"

namespace localization {
    namespace util {

        double GetTimeInterval(clock_t start_time) {
            clock_t current_time = clock();
            return (double) (current_time - start_time) / CLOCKS_PER_SEC;
        }

        double GetTimeInterval(clock_t start_time, clock_t end_time) {
            return (double) (end_time - start_time) / CLOCKS_PER_SEC;
        }

    } // namespace util
} // namespace localization
