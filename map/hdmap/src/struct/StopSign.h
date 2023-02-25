//
// Created by luyifan on 19-6-15.
//

#ifndef HDMAP_STOPSIGN_H
#define HDMAP_STOPSIGN_H

#include "Geometry.h"

namespace hdmap{
  struct StopSign{
    d_string id;
    std::vector<Curve> stop_line;
    std::vector<d_string> overlap_ids;
    d_double stop_time;

    enum StopType{
      UNKNOWN = 0,
      ONE_WAY = 1,
      TWO_WAY = 2,
      THREE_WAY = 3,
      FOUR_WAY = 4,
      ALL_WAY = 5
    };
    xsd::Attribute<StopType> type;
  };
}
#endif //HDMAP_STOPSIGN_H
