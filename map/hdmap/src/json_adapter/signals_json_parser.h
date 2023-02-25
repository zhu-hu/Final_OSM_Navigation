//
// Created by Chen Xiaofeng on 19-10-30.
//

#ifndef HDMAP_SIGNALS_JSON_PARSER_H
#define HDMAP_SIGNALS_JSON_PARSER_H

#include <jsoncpp/json/json.h>
#include "../struct/Attribute.h"
#include "../struct/Map.h"
#include "common/utils/find_path.h"
#include "util_json_parser.h"

namespace hdmap{
  namespace adapter{
    class SignalsJsonParser {
    public:
      static bool Parse(const Json::Value &json_node,
                 std::vector<Signal>& signals);
    };
  }
}

#endif //HDMAP_SIGNALS_JSON_PARSER_H
