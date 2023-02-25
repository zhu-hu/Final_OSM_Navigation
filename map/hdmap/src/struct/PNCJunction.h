//
// Created by luyifan on 19-6-15.
//

#ifndef HDMAP_PNCJUNCTION_H
#define HDMAP_PNCJUNCTION_H

#include "Geometry.h"

namespace hdmap{
  struct PNCJunction{
    d_string id;
    Attribute<Polygon> polygon;
    std::vector<d_string> overlap_ids;
  };
}

#endif //HDMAP_PNCJUNCTION_H
