//
// Created by Chen Xiaofeng on 19-10-30.
//

#ifndef HDMAP_STOP_SIGNS_JSON_PARSER_H
#define HDMAP_STOP_SIGNS_JSON_PARSER_H

#include <jsoncpp/json/json.h>
#include "../struct/Attribute.h"
#include "../struct/Map.h"
#include "common/utils/find_path.h"
#include "util_json_parser.h"

namespace hdmap{
  namespace adapter{
    class StopSignsJsonParser {
    public:
      static bool Parse(const Json::Value &json_node,
                        std::vector<StopSign>& stop_signs);
    };
  }
}

#endif //HDMAP_STOP_SIGNS_JSON_PARSER_H
