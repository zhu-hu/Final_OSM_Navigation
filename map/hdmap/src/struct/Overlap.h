//
// Created by luyifan on 19-6-15.
//

#ifndef HDMAP_OVERLAP_H
#define HDMAP_OVERLAP_H

#include "Geometry.h"

namespace hdmap{
  struct ObjectOverlapInfo{
    enum Type{
      Lane,
      Crosswalk,
      Signal,
      StopSign,
      Junction,
      Yield,
      ClearArea,
      SpeedBump,
      ParkingSpace,
      PNCJunction
    };
    d_string id;
    Attribute<Type> type;
    d_double start_s;
    d_double end_s;
    d_bool is_merge;
    d_string region_overlap_id;
  };

  /*
  struct LaneOverlapInfo{
    double start_s;
    double end_s;
    bool is_merge;
    int region_overlap_id;
  };

  struct CrosswalkOverlapInfo{
    int region_overlap_id;
  };

  struct SignalOverlapInfo{

  };

  struct StopSignOverlapInfo{

  };

  struct JunctionOverlapInfo{

  };

  struct YieldOverlapInfo{

  };

  struct ClearAreaOverlapInfo{

  };

  struct SpeedBumpOverlapInfo{

  };

  struct ParkingSpaceOverlapInfo{

  };

  struct PNCJunctionOverlapInfo{

  };
  */
  struct RegionOverlapInfo{
    d_int id;
    std::vector<Attribute<Polygon> > polygons;
  };

  // Here, the "overlap" includes any pair of objects on the map
  // (e.g. lanes, junctions, and crosswalks).
  struct Overlap{
    enum Type{
      LANE,
      STOP_SIGN,
      SIGNAL
    };
    d_string id;
    d_string object1_id;
    xsd::Attribute<Type> object1_type;
    d_string object2_id;
    xsd::Attribute<Type> object2_type;
    d_double start_s;
    d_double end_s;
  };
}

#endif //HDMAP_OVERLAP_H
