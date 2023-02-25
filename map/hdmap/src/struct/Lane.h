//
// Created by luyifan on 19-6-15.
//

#ifndef HDMAP_LANE_H
#define HDMAP_LANE_H

#include "Geometry.h"

namespace hdmap{
  // previous LaneBoundary
  // struct LaneBoundaryType{
  //   enum Type{
  //     UNKNOWN,
  //     DOTTED_YELLOW,
  //     DOTTED_WHITE,
  //     SOLID_YELLOW,
  //     SOLID_WHITE,
  //     DOUBLE_YELLOW,
  //     CURB
  //   };
  //   // Offset relative to the starting point of boundary
  //   d_double s;
  //   // support multiple types
  //   std::vector<Type> types;
  // };

  // struct LaneBoundary{
  //   xsd::Attribute<Curve> curve;
  //   d_double length;
  //   // indicate whether the lane boundary exists in real world
  //   d_bool bVirtual;
  //   // in ascending order of s
  //   std::vector<LaneBoundaryType> types;//每一条路沿可能由多个不同类型的部分组成
  // };

  // new LaneBoundary for json map
  struct LaneBoundary {
    enum Type{
      CURB,
      DOTTED_WHITE,
      DOTTED_YELLOW,
      SOLID_WHITE,
      SOLID_YELLOW
    };
    d_double s;
    xsd::Attribute<Type> type;
    d_double length;
  };

  // Association between central point to closest boundary.
  struct LaneSampleAssociation {
    d_double s;
    d_double width;//s处，center_line到车道边线的距离
  };

  // A lane is part of a roadway, that is designated for use by a single line of vehicles.
  // Most public roads (include highways) have more than two lanes.
  struct Lane {
    d_string id;
    d_string road_id;
    // Central lane as reference trajectory, not necessary to be the geometry central
    xsd::Attribute<Curve> central_curve;
  
    //in meters
    d_double length;
    //speed limit of the lane, in m/s
    d_double speed_limit;

    //speed expectation km/h
    d_double speed_expect;

    //std::vector<d_string> overlap_ids;
    // All lanes can be driving into (or from).
    std::vector<d_string> predecessor_ids;
    std::vector<d_string> successor_ids;
    std::vector<d_string> lefts;
    std::vector<d_string> rights;
    std::vector<d_string> left_reverse_ids;
    // Neighbor lanes on the same direction.
    // std::vector<d_string> left_neighbor_forward_lane_ids;
    // std::vector<d_string> right_neighbor_forward_lane_ids;

    enum LaneType{
      NORMAL_ROAD,
      JUNCTION_ROAD,
      COUNTRY_ROAD,
      SLOW_ROAD,
      PARKING_ROAD,
      IDLING_ROAD,
      HIGHWAY_ROAD,
      STURN_ROAD
    };
    xsd::Attribute<LaneType> type;

    enum LaneTurn{
      NO_TURN,
      LEFT_TURN,
      RIGHT_TURN
    };
    xsd::Attribute<LaneTurn> turn;

    //std::vector<d_string> left_neighbor_reverse_lane_ids;
    //std::vector<d_string> right_neighbor_reverse_lane_ids;

    //d_string junction_id;

    // Association between central point to closest boundary.
    std::vector<LaneSampleAssociation> left_sample;
    std::vector<LaneSampleAssociation> right_sample;

    // Lane boundary
    std::vector<LaneBoundary> left_boundary;
    std::vector<LaneBoundary> right_boundary;

    // overlaps
    std::vector<d_string> overlap_ids;

    //enum LaneDirection{
    //  FORWARD,
    //  BACKWARD,
    //  BIDIRECTION
    //};
    //xsd::Attribute<LaneDirection> direction;

    // Association between central point to closest road boundary.
    //std::vector<LaneSampleAssociation> left_road_sample;
    //std::vector<LaneSampleAssociation> right_road_sample;

    //std::vector<d_string> self_reverse_lane_ids;
  };
}

#endif //HDMAP_LANE_H
