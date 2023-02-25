//
// Created by Chen Xiaofeng on 19-10-30.
//

#include "stop_signs_json_parser.h"

namespace hdmap{
  namespace adapter{
    bool StopSignsJsonParser::Parse(const Json::Value &json_node,
                                    std::vector<StopSign>& stop_signs) {
      for(const auto & stop_sign_item : json_node){
        StopSign stop_sign;

        // get id
        if(!stop_sign_item["id"].empty()) stop_sign.id = stop_sign_item["id"].asString();
        else AERROR << "stop sign has no id";

        // get stop time
        if(!stop_sign_item["stop_time"].empty()) stop_sign.stop_time = stop_sign_item["stop_time"].asDouble();
        else AERROR << "stop sign has no stop time";

        // get stop line curve
        const auto points_node = stop_sign_item["stop_line_points"];
        std::vector<common::PointENU> points;
        if(!UtilJsonParser::ParsePoints(points_node, points)){
          AERROR << "stop sign "<<stop_sign.id<<" points are invalid";
          return -1;
        }
        Curve curve;
        CurveSegment curve_segment;
        curve_segment.line.create();
        for (const auto& pt : points){
          curve_segment.line->points.emplace_back(pt);
        }
        curve.segments.emplace_back(curve_segment);
        stop_sign.stop_line.emplace_back(curve);

        // get overlaps
        for(const auto & overlap_id : stop_sign_item["overlap_id"]){
          string id_tmp = overlap_id.asString();
          d_string tmp;
          tmp = id_tmp;
          stop_sign.overlap_ids.emplace_back(tmp);
        }

        stop_signs.emplace_back(stop_sign);
      }
      return true;
    }
  }
}