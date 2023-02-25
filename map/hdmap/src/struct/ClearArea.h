//
// Created by luyifan on 19-6-15.
//

#ifndef HDMAP_CLEARAREA_H
#define HDMAP_CLEARAREA_H

#include "Geometry.h"

namespace hdmap{
  struct ClearArea{
    d_string id;
    std::vector<d_string> overlap_ids;
    Attribute<Polygon> polygon;
  };
}

#endif //HDMAP_CLEARAREA_H
