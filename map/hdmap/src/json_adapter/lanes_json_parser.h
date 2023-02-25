//
// Created by Chen Xiaofeng on 19-10-20.
//

#ifndef HDMAP_LANES_JSON_PARSER_H
#define HDMAP_LANES_JSON_PARSER_H

#include <jsoncpp/json/json.h>
#include "../struct/Attribute.h"
#include "../struct/Map.h"
#include "common/utils/find_path.h"
#include "util_json_parser.h"

namespace hdmap{
  namespace adapter{
    class LanesJsonParser {
    public:
      static bool Parse(const Json::Value &json_node,
                        const std::string& road_id,
                        std::vector<Lane>& lanes);
      static bool to_lane_type(const std::string& type,
                               Lane::LaneType & lane_type);
      static bool to_turn_type(const std::string& turn,
                               Lane::LaneTurn & lane_turn);
      static bool to_boundary_type(const std::string& boundary,
                                   LaneBoundary::Type & boundary_type);
    };
  }
}

#endif //HDMAP_LANES_JSON_PARSER_H
