//
// Created by luyifan on 19-6-15.
//

#ifndef HDMAP_SPEEDCONTROL_H
#define HDMAP_SPEEDCONTROL_H

#include "Geometry.h"

namespace hdmap{
  struct SpeedControl{
    d_string name;
    Attribute<Polygon> polygon;
    d_double speed_limit;
  };

  struct SpeedControls{
    std::vector<SpeedControl> controls;
  };
}

#endif //HDMAP_SPEEDCONTROL_H
