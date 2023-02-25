//
// Created by luyifan on 19-6-15.
//

#ifndef HDMAP_YIELDSIGN_H
#define HDMAP_YIELDSIGN_H

#include "Geometry.h"

namespace hdmap{
  // A yield indicates that each driver must prepare to stop if necessary to let a
  // driver on another approach proceed.
  // A driver who stops or slows down to let another vehicle through has yielded
  // the right of way to that vehicle.
  struct YieldSign{
    d_string id;
    std::vector<Curve> stop_line;
    std::vector<d_string> overlap_ids;
  };
}
#endif //HDMAP_YIELDSIGN_H
