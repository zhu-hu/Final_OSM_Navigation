//
// Created by localization on 11/4/19.
//

#ifndef SRC_TWISTSTAMPED_H
#define SRC_TWISTSTAMPED_H

#include "Twist.h"

namespace localization{
    struct TwistStamped{
        double timestamp;
        Twist twist;
    };
} // namespace localization

#endif //SRC_TWISTSTAMPED_H
