//
// Created by Chen Xiaofeng on 19-10-30.
//

#include "junctions_json_parser.h"

namespace hdmap{
  namespace adapter{
    bool JunctionsJsonParser::Parse(const Json::Value &json_node,
                                    std::vector<Junction>& junctions) {
      for(const auto & junction_item : json_node){
        Junction junction;

        // get id
        if(!junction_item["id"].empty()) junction.id = junction_item["id"].asString();
        else AERROR << "junction has no id";

        // get polygon
        const auto points_node = junction_item["ploygon"];
        std::vector<common::PointENU> points;
        if(!UtilJsonParser::ParsePoints(points_node, points)){
          AERROR << "junction "<<junction.id<<" points are invalid";
          return -1;
        }
        junction.polygon.create();
        for (const auto& pt : points){
          junction.polygon->points.emplace_back(pt);
        }

        // get overlaps
        for(const auto & overlap_id : junction_item["lane_id"]){
          string id_tmp = overlap_id.asString();
          d_string tmp;
          tmp = id_tmp;
          junction.overlap_ids.emplace_back(tmp);
        }

        junctions.emplace_back(junction);
      }
      return true;
    }
  }
}