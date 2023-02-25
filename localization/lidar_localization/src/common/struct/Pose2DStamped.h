//
// Created by localization on 11/5/19.
//

#ifndef SRC_POSE2DSTAMPED_H
#define SRC_POSE2DSTAMPED_H

#include "Pose2D.h"

namespace localization{
    struct Pose2DStamped{
        double timestamp;
        Pose2D pose;
    };
} // namespace localization

#endif //SRC_POSE2DSTAMPED_H
