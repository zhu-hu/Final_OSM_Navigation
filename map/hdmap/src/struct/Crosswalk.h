//
// Created by luyifan on 19-6-15.
//

#ifndef HDMAP_CROSSWALK_H
#define HDMAP_CROSSWALK_H

#include "Geometry.h"

namespace hdmap{
  // Crosswalk is a place designated for pedestrians to cross a road.
  struct Crosswalk{
    d_string id;
    std::vector<d_string> overlap_ids;
    Attribute<Polygon> polygon;
  };
}

#endif //HDMAP_CROSSWALK_H
