//
// Created by Chen Xiaofeng on 19-10-30.
//

#include "overlaps_json_parser.h"

namespace hdmap{
  namespace adapter{
    bool OverlapsJsonParser::Parse(const Json::Value &json_node,
                                   std::vector<Overlap>& overlaps) {
      for(const auto & overlap_item : json_node){
        Overlap overlap;

        // get id
        if(!overlap_item["id"].empty()) overlap.id = overlap_item["id"].asString();
        else AERROR << "overlap has no id";

        // get object id
        if(!overlap_item["object1"]["id"].empty()) overlap.object1_id = overlap_item["object1"]["id"].asString();
        else AERROR << "overlap object1 has no id";
        if(!overlap_item["object2"]["id"].empty()) overlap.object2_id = overlap_item["object2"]["id"].asString();
        else AERROR << "overlap object2 has no id";

        // get object type
        std::string overlap_temp = overlap_item["object1"]["type"].asString();
        Overlap::Type overlap_type;
        if(!to_overlap_type(overlap_temp, overlap_type)){
            return -1;
        }
        overlap.object1_type = overlap_type;
        overlap_temp = overlap_item["object2"]["type"].asString();
        if(!to_overlap_type(overlap_temp, overlap_type)){
            return -1;
        }
        overlap.object2_type = overlap_type;

        // get lane overlap info
        overlap.start_s = overlap_item["object1"]["lane_overlap_info"]["start_s"].asDouble();
        overlap.end_s = overlap_item["object1"]["lane_overlap_info"]["end_s"].asDouble();

        overlaps.emplace_back(overlap);
      }
      return true;
    }

    bool OverlapsJsonParser::to_overlap_type(const std::string& overlap,
                                             Overlap::Type & overlap_type) {
        if (overlap == "lane") {
          overlap_type = hdmap::Overlap::Type::LANE;
        } else if (overlap == "stop_sign") {
          overlap_type = hdmap::Overlap::Type::STOP_SIGN;
        } else if (overlap == "signal") {
          overlap_type = hdmap::Overlap::Type::SIGNAL;
        } else {
          AERROR << "Error or unsupport road turn" << overlap_type;
          return false;
        }
        return true;
    }
  }
}