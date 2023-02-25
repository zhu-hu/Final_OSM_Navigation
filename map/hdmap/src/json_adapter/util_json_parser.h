//
// Created by Chen Xiaofeng on 19-10-20.
//

#ifndef HDMAP_UTIL_JSON_PARSER_H
#define HDMAP_UTIL_JSON_PARSER_H
#include <jsoncpp/json/json.h> 
#include "../struct/Attribute.h"
#include "common/utils/find_path.h"
#include "../struct/Map.h"
#include "common/utils/wgs84_to_utm.h"
namespace hdmap{
  namespace adapter{
    extern double GLOBAL_ZERO_X;
    extern double GLOBAL_ZERO_Y;

    class UtilJsonParser {
    public:
      static bool ParsePoints(const Json::Value &json_node,
                              std::vector<common::PointENU>& pts);
      static bool ParseLink(const Json::Value &json_node,
                            std::vector<d_string>& successor);

      static std::string ToUpper(const std::string& s);
    };
  }
}

#endif //HDMAP_UTIL_JSON_PARSER_H
