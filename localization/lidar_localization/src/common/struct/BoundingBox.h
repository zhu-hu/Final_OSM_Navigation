//
// Created by localization on 11/5/19.
//

#ifndef SRC_BOUNDINGBOX_H
#define SRC_BOUNDINGBOX_H

#include "Point.h"

namespace localization{
    struct BoundingBox{
        Point min;
        Point max;
    };
} // namespace localization

#endif //SRC_BOUNDINGBOX_H
