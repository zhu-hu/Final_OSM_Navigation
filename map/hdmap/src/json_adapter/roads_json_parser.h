//
// Created by Chen Xiaofeng on 19-10-20.
//

#ifndef HDMAP_ROADS_JSON_PARSER_H
#define HDMAP_ROADS_JSON_PARSER_H

#include <jsoncpp/json/json.h> 
#include "../struct/Attribute.h"
#include "common/utils/find_path.h"
#include "../struct/Map.h"
#include "util_json_parser.h"

namespace hdmap{
  namespace adapter{
    class RoadsJsonParser{
    public:
      static bool Parse(const Json::Value& json_node,
                        std::vector<Road>& roads);
    };
  }
}

#endif //HDMAP_ROADS_JSON_PARSER_H
