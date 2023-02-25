//
// Created by luyifan on 19-6-15.
//

#ifndef HDMAP_JUNCTION_H
#define HDMAP_JUNCTION_H

#include "Geometry.h"

// A junction is the junction at-grade of two or more roads crossing.
namespace hdmap{
  struct Junction{
    d_string id;
    std::vector<d_string> overlap_ids;
    xsd::Attribute<Polygon> polygon;
  };
}

#endif //HDMAP_JUNCTION_H
