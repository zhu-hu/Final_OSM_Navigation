//
// Created by luyifan on 19-6-15.
//

#ifndef HDMAP_SPEEDBUMP_H
#define HDMAP_SPEEDBUMP_H

#include "Geometry.h"

namespace hdmap{
  struct SpeedBump{
    d_string id;
    std::vector<d_string> overlap_ids;
    std::vector<Curve> position;
  };
}

#endif //HDMAP_SPEEDBUMP_H
