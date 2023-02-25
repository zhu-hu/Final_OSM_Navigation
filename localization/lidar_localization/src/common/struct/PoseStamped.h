//
// Created by localization on 11/4/19.
//

#ifndef SRC_POSESTAMPED_H
#define SRC_POSESTAMPED_H

#include "Pose.h"

namespace localization{
    struct PoseStamped{
        double timestamp;
        Pose pose;
    };
} // namespace localization

#endif //SRC_POSESTAMPED_H
