//
// Created by localization on 11/4/19.
//

#ifndef SRC_TWIST_H
#define SRC_TWIST_H

#include "Vector3.h"

namespace localization{
    struct Twist{
        Vector3 linear;
        Vector3 angular;
    };
} // namespace localization

#endif //SRC_TWIST_H
