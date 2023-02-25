//
// Created by luyifan on 19-6-17.
//

#ifndef HDMAP_MAP_H
#define HDMAP_MAP_H

#include "ClearArea.h"
#include "Crosswalk.h"
#include "Junction.h"
#include "Lane.h"
#include "Overlap.h"
#include "Signal.h"
#include "SpeedBump.h"
#include "StopSign.h"
#include "YieldSign.h"
#include "Road.h"
#include "ParkingSpace.h"
#include "PNCJunction.h"

namespace hdmap{
  struct Projection{
    // PROJ.4 setting:
    // "+proj=tmerc +lat_0={origin.lat} +lon_0={origin.lon} +k={scale_factor}
    // +ellps=WGS84 +no_defs"
    d_string proj;
  };

  struct Header{
    d_string version;
    d_string date;
    d_string projection;
    d_string district;
    d_string generation;
    d_string rev_major;
    d_string rev_minor;
    d_double left;
    d_double top;
    d_double right;
    d_double bottom;
    d_string vendor;
  };

  struct Map{
    xsd::Attribute<Header> header;
    std::vector<Road> roads;
    std::vector<StopSign> stop_signs;
    std::vector<Signal> signals;
    std::vector<Junction> junctions;
    std::vector<Overlap> overlaps;
  };
}

#endif //HDMAP_MAP_H
