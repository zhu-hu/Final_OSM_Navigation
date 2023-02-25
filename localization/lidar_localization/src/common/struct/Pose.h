//
// Created by localization on 11/4/19.
//

#ifndef SRC_POSE_H
#define SRC_POSE_H

#include "Point.h"
#include "Quaternion.h"

namespace localization{
    struct Pose{
        Point position;
        Quaternion orientation;
    };
} // namespace localization

#endif //SRC_POSE_H
