//
// Created by Chen Xiaofeng on 19-10-30.
//

#ifndef HDMAP_OVERLAPS_JSON_PARSER_H
#define HDMAP_OVERLAPS_JSON_PARSER_H

#include <jsoncpp/json/json.h>
#include "../struct/Attribute.h"
#include "../struct/Map.h"
#include "common/utils/find_path.h"
#include "util_json_parser.h"

namespace hdmap{
  namespace adapter{
    class OverlapsJsonParser {
    public:
      static bool Parse(const Json::Value &json_node,
                 std::vector<Overlap>& overlaps);
      static bool to_overlap_type(const std::string& overlap,
                               Overlap::Type & overlap_type);
    };
  }
}

#endif //HDMAP_OVERLAPS_JSON_PARSER_H
