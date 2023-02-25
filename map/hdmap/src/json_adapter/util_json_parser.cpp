//
// Created by Chen Xiaofeng on 19-10-20.
//

#include "util_json_parser.h"

namespace hdmap{
  namespace adapter{
    double GLOBAL_ZERO_X;
    double GLOBAL_ZERO_Y;

    std::string UtilJsonParser::ToUpper(const std::string& s) {
      std::string value = s;
      std::transform(value.begin(), value.end(), value.begin(),
                     [](unsigned char c) { return std::toupper(c); });

      return value;
    }

    bool UtilJsonParser::ParsePoints(const Json::Value &json_node,
                                     std::vector<common::PointENU>& pts) {
      auto sub_node = json_node;
      CHECK(!sub_node.empty());
      for(const auto & json_pt : json_node){
        common::PointENU pt;
        double latitude = json_pt["latitude"].asDouble();
        double longitude = json_pt["longitude"].asDouble();
        double x_temp, y_temp;
        char zone;
        LLtoUTM(latitude,longitude,y_temp,x_temp,&zone,GLOBAL_ZERO_X,GLOBAL_ZERO_Y);
        pt.x = x_temp;
        pt.y = y_temp;
        pt.z = 0;
        pts.emplace_back(pt);
      }
      return true;
    }

    bool UtilJsonParser::ParseLink(const Json::Value &json_node,
                                  std::vector<d_string>& successor) {
      if(!json_node["next_id"].isString()){
        for(const auto & id : json_node["next_id"]){
          int id_int = id.asInt();
          string id_temp = std::to_string(id_int);
          d_string tmp;
          tmp = id_temp;
          successor.emplace_back(tmp);
        }
      }
      return true;
    }
  }
}