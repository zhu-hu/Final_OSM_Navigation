//
// Created by luyifan on 19-6-15.
//

#ifndef HDMAP_ROAD_H
#define HDMAP_ROAD_H

#include "Geometry.h"
#include "Lane.h"
#include "Signal.h"
#include "StopSign.h"

namespace hdmap{
  // The road is a collection of traffic elements, such as lanes, road boundary
  // It provides general information about the road.
  struct Road{
    d_string id;
    std::vector<Lane> lanes;
    std::vector<d_string> predecessor_ids;
    std::vector<d_string> successor_ids;
    std::vector<common::PointENU> topology;
    std::vector<common::PointENU> left_boundary;
    std::vector<common::PointENU> right_boundary;
    std::vector<Signal> signals;
    // std::vector<StopSign> stop_signs;
    enum Type{
      UNKNOWN,
      HIGHWAY,
      CITY_ROAD,
      PARK,
      JUNCTION
    };

    Attribute<Type> type;
  };
}

#endif //HDMAP_ROAD_H

//13946