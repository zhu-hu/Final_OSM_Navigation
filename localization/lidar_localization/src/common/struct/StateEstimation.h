//
// Created by localization on 11/4/19.
//

#ifndef SRC_STATEESTIMATION_H
#define SRC_STATEESTIMATION_H

#include "Pose.h"
#include "Twist.h"

namespace localization{
    struct StateEstimation{
        double timestamp;
        Pose pose;
        Twist twist;
    };
} // namespace localization

#endif //SRC_STATEESTIMATION_H
