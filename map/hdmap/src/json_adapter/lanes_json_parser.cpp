//
// Created by Chen Xiaofeng on 19-10-20.
//

#include "lanes_json_parser.h"

namespace {
  double ToMPS(double speed) { return speed * 1000.0 / 3600.0; }
  bool IsReferenceLane(int lane_id) { return lane_id == 0; }
};  // namespace


namespace hdmap{
  namespace adapter{
    bool LanesJsonParser::Parse(const Json::Value &json_node,
                                const std::string& road_id,
                                std::vector<Lane>& lanes) {
      Lane lane;
      // get id
      if(!json_node["id"].empty()) lane.id = json_node["id"].asString();
      else AERROR << "lane has no id";

      // get type
      std::string type = json_node["type"].asString();
      Lane::LaneType tmp_type;
      if(!to_lane_type(type, tmp_type)){
          return -1;
      }
      lane.type = tmp_type;

      // get turn
      std::string turn = json_node["turn"].asString();
      Lane::LaneTurn tmp_turn;
      if(!to_turn_type(turn, tmp_turn)){
          return -1;
      }
      lane.turn = tmp_turn;

      // get central curve
      const auto points_node = json_node["central_curve_points"];
      std::vector<common::PointENU> points;
      if(!UtilJsonParser::ParsePoints(points_node, points)){
        AERROR << "road "<<road_id<<" lane "<<lane.id<<" points are invalid";
        return -1;
      }
      lane.central_curve.create();
      CurveSegment curve;
      curve.line.create();
      for (const auto& pt : points){
        curve.line->points.emplace_back(pt);
      }
      lane.central_curve->segments.emplace_back(curve);

      // get length
      lane.length = json_node["length"].asDouble();

      // get links
      Json::Value links_node = json_node["link"];
      for(const auto & successor_id : links_node["successor_id"]){
        string id_tmp = successor_id.asString();
        d_string tmp;
        tmp = id_tmp;
        lane.successor_ids.emplace_back(tmp);
      }
      for(const auto & predecessor_id : links_node["predecessor_id"]){
        string id_tmp = predecessor_id.asString();
        d_string tmp;
        tmp = id_tmp;
        lane.predecessor_ids.emplace_back(tmp);
      }
      if(!links_node["left_lane_id"].empty()){
        string id_tmp = links_node["left_lane_id"].asString();
        d_string tmp;
        tmp = id_tmp;
        lane.lefts.emplace_back(tmp);
      }
      if(!links_node["right_lane_id"].empty()){
        string id_tmp = links_node["right_lane_id"].asString();
        d_string tmp;
        tmp = id_tmp;
        lane.rights.emplace_back(tmp);
      }
      if(!links_node["left_reverse_lane_id"].empty()){
        string id_tmp = links_node["left_reverse_lane_id"].asString();
        d_string tmp;
        tmp = id_tmp;
        lane.left_reverse_ids.emplace_back(tmp);
      }

      // get left and right samples
      for(const auto & left_sample : json_node["left_sample"]){
        LaneSampleAssociation left_sample_tmp;
        left_sample_tmp.s = left_sample["s"].asDouble();
        left_sample_tmp.width = left_sample["width"].asDouble();
        lane.left_sample.emplace_back(left_sample_tmp);
      }
      for(const auto & right_sample : json_node["right_sample"]){
        LaneSampleAssociation right_sample_tmp;
        right_sample_tmp.s = right_sample["s"].asDouble();
        right_sample_tmp.width = right_sample["width"].asDouble();
        lane.right_sample.emplace_back(right_sample_tmp);
      }

      // get left and right boundary
      for(const auto & left_boundary : json_node["left_boundary"]){
        LaneBoundary left_boundary_tmp;
        left_boundary_tmp.s = left_boundary["s"].asDouble();
        std::string boundary_tmp = left_boundary["type"].asString();
        LaneBoundary::Type type_tmp;
        if(!to_boundary_type(boundary_tmp, type_tmp)){
          return -1;
        }
        left_boundary_tmp.type = type_tmp;
        left_boundary_tmp.length = left_boundary["length"].asDouble();
        lane.left_boundary.emplace_back(left_boundary_tmp);
      }
      for(const auto & right_boundary : json_node["right_boundary"]){
        LaneBoundary right_boundary_tmp;
        right_boundary_tmp.s = right_boundary["s"].asDouble();
        std::string boundary_tmp = right_boundary["type"].asString();
        LaneBoundary::Type type_tmp;
        if(!to_boundary_type(boundary_tmp, type_tmp)){
          return -1;
        }
        right_boundary_tmp.type = type_tmp;
        right_boundary_tmp.length = right_boundary["length"].asDouble();
        lane.right_boundary.emplace_back(right_boundary_tmp);
      }

      // get overlaps
      for(const auto & overlap_id : json_node["overlap_id"]){
        string id_tmp = overlap_id.asString();
        d_string tmp;
        tmp = id_tmp;
        lane.overlap_ids.emplace_back(tmp);
      }

      if(!json_node["expect_speed"].empty()){
        double expect_speed = json_node["expect_speed"].asDouble();
        lane.speed_expect = expect_speed;
      }

      lanes.emplace_back(lane);
      return true;
    }

    bool LanesJsonParser::to_lane_type(const std::string& type,
                                      Lane::LaneType & lane_type) {
      if (type == "NORMAL_ROAD") {
        lane_type = hdmap::Lane::LaneType::NORMAL_ROAD;
      } else if (type == "JUNCTION_ROAD") {
        lane_type = hdmap::Lane::LaneType::JUNCTION_ROAD;
      } else if (type == "COUNTRY_ROAD") {
        lane_type = hdmap::Lane::LaneType::COUNTRY_ROAD;
      } else if (type == "SLOW_ROAD") {
        lane_type = hdmap::Lane::LaneType::SLOW_ROAD;
      } else if (type == "HIGHWAY_ROAD") {
        lane_type = hdmap::Lane::LaneType::HIGHWAY_ROAD;
      } else if (type == "STURN_ROAD") {
        lane_type = hdmap::Lane::LaneType::STURN_ROAD;
      } else if (type == "IDLING_ROAD") {
        lane_type = hdmap::Lane::LaneType::IDLING_ROAD;
      } else if (type == "PARKING_ROAD") {
        lane_type = hdmap::Lane::LaneType::PARKING_ROAD;
      } else {
        AERROR << "Error or unsupport road type" << type;
        return false;
      }
      return true;
    }

    bool LanesJsonParser::to_turn_type(const std::string& turn,
                                      Lane::LaneTurn & lane_turn) {
      if (turn == "NO_TURN") {
        lane_turn = hdmap::Lane::LaneTurn::NO_TURN;
      } else if (turn == "LEFT_TURN") {
        lane_turn = hdmap::Lane::LaneTurn::LEFT_TURN;
      } else if (turn == "RIGHT_TURN") {
        lane_turn = hdmap::Lane::LaneTurn::RIGHT_TURN;
      } else {
        AERROR << "Error or unsupport road turn" << turn;
        return false;
      }
      return true;
    }

    bool LanesJsonParser::to_boundary_type(const std::string& boundary,
                                      LaneBoundary::Type & boundary_type) {
      if (boundary == "CURB") {
        boundary_type = hdmap::LaneBoundary::Type::CURB;
      } else if (boundary == "DOTTED_WHITE") {
        boundary_type = hdmap::LaneBoundary::Type::DOTTED_WHITE;
      } else if (boundary == "DOTTED_YELLOW") {
        boundary_type = hdmap::LaneBoundary::Type::DOTTED_YELLOW;
      } else if (boundary == "SOLID_WHITE") {
        boundary_type = hdmap::LaneBoundary::Type::SOLID_WHITE;
      } else if (boundary == "SOLID_YELLOW") {
        boundary_type = hdmap::LaneBoundary::Type::SOLID_YELLOW;
      } else {
        AERROR << "Error or unsupport boundary type" << boundary;
        return false;
      }
      return true;
    }
  }
}