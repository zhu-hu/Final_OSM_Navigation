//
// Created by luyifan on 19-6-15.
//

#ifndef HDMAP_SIGNAL_H
#define HDMAP_SIGNAL_H

#include "Geometry.h"

namespace hdmap{
  struct Signal{
    enum Type{
      ARROW
    };

    d_string id;
    //TODO: add orientation
    //std::vector<d_string> overlap_ids;
    xsd::Attribute<Type> type;
    //stop line
    std::vector<Curve> stop_line;
    xsd::Attribute<common::PointENU> position;
    d_double latitude;
    d_double longitude;
    d_double height;
    std::vector<d_string> overlap_ids;
  };
}

#endif //HDMAP_SIGNAL_H
