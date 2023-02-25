//
// Created by luyifan on 19-6-12.
//

#ifndef HDMAP_LOG_H
#define HDMAP_LOG_H

#include "glog/logging.h"

#define ADEBUG VLOG(4) << "[DEBUG] "
#define AINFO LOG(INFO)
#define AWARN LOG(WARNING)
#define AERROR LOG(ERROR)
#define AFATAL LOG(FATAL)

// LOG_IF
#define INFO_IF(cond) LOG_IF(INFO, cond)
#define WARN_IF(cond) LOG_IF(WARNING, cond)
#define ERROR_IF(cond) LOG_IF(ERROR, cond)

// LOG_EVERY_N
#define INFO_EVERY(freq) LOG_EVERY_N(INFO, freq)
#define WARN_EVERY(freq) LOG_EVERY_N(WARNING, freq)
#define ERROR_EVERY(freq) LOG_EVERY_N(ERROR, freq)

#define RETURN_IF(condition)                   \
    if (condition) {                           \
        WARN << #condition << " is not met."; \
        return;                                \
    }

#define RETURN_IF_NULL(ptr)          \
  if (ptr == nullptr) {              \
    AWARN << #ptr << " is nullptr."; \
    return;                          \
  }

#define RETURN_VAL_IF(condition, val)      \
  if (condition) {                         \
    AWARN << #condition << " is not met."; \
    return val;                            \
  }

#define RETURN_VAL_IF_NULL(ptr, val) \
  if (ptr == nullptr) {              \
    AWARN << #ptr << " is nullptr."; \
    return val;                      \
  }
#endif //HDMAP_LOG_H
