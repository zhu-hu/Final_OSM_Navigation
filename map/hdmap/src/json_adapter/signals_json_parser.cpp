//
// Created by Chen Xiaofeng on 19-10-30.
//

#include "signals_json_parser.h"

namespace hdmap{
  namespace adapter{
    bool SignalsJsonParser::Parse(const Json::Value &json_node,
                                  std::vector<Signal>& signals) {
      for(const auto & signal_item : json_node){
        Signal signal;

        // get id
        if(!signal_item["id"].empty()) signal.id = signal_item["id"].asString();
        else AERROR << "signal has no id";

        // get location
        if(!signal_item["location"]["latitude"].empty()) signal.latitude = signal_item["location"]["latitude"].asDouble();
        else AERROR << "signal has no latitude";

        if(!signal_item["location"]["longitude"].empty()) signal.longitude = signal_item["location"]["longitude"].asDouble();
        else AERROR << "signal has no longitude";

        if(!signal_item["location"]["height"].empty()) signal.height = signal_item["location"]["height"].asDouble();
        else AERROR << "signal has no height";

        // get overlaps
        for(const auto & overlap_id : signal_item["overlap_id"]){
          string id_tmp = overlap_id.asString();
          d_string tmp;
          tmp = id_tmp;
          signal.overlap_ids.emplace_back(tmp);
        }

        signals.emplace_back(signal);
      }
      return true;
    }
  }
}