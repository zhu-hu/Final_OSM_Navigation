//
// Created by Chen Xiaofeng on 19-10-20.
//

#include "roads_json_parser.h"
#include "lanes_json_parser.h"

namespace hdmap{
  namespace adapter{
    bool RoadsJsonParser::Parse(const Json::Value& json_node,
                                std::vector<Road>& roads) {
      for(const auto & road_item : json_node){
        Road road;
        if(!road_item["id"].empty()) road.id = road_item["id"].asString();
        else AERROR << "road has no id";
        road.type.create();
        Road::Type road_type = hdmap::Road::UNKNOWN;
        road.type = road_type;

        if(!LanesJsonParser::Parse(road_item, *road.id, road.lanes)){
              AERROR << "road " << road.id << "lanes invalid";
              return false;
        }
        roads.emplace_back(road);
      }
      return true;
    }
  }
}