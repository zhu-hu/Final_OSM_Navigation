//
// Created by luyifan on 19-6-15.
//

#ifndef HDMAP_PARKINGSPACE_H
#define HDMAP_PARKINGSPACE_H

#include "Geometry.h"

namespace hdmap{
  // ParkingSpace is a place designated to park a car.
  struct ParkingSpace{
    d_string id;
    Attribute<Polygon> polygon;
    std::vector<d_string> overlap_ids;
    d_double heading;
  };

  // ParkingLot is a place for parking cars.
  struct ParkingLot{
    d_string id;
    Attribute<Polygon> polygon;
    std::vector<d_string> overlap_ids;
  };
}

#endif //HDMAP_PARKINGSPACE_H
