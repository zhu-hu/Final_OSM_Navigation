//
// Created by localization on 11/5/19.
//

#ifndef SRC_POSE3DSTAMPED_H
#define SRC_POSE3DSTAMPED_H

#include "Pose3D.h"

namespace localization{
    struct Pose3DStamped{
        double timestamp;
        Pose3D pose;
    };
} // namespace localization

#endif //SRC_POSE3DSTAMPED_H
