//
// Created by Chen Xiaofeng on 19-10-20.
//

#include "json_adapter.h"

namespace hdmap{
  namespace adapter{
    bool JsonAdapter::LoadData(const std::string &filename, Map *map) {
      Json::Value raw_map;
      Json::Reader reader;

      std::ifstream ifs(filename);
      if(ifs.fail()){
          AERROR << "fail to load file " << filename;
          return false;
      }
      if(!reader.parse(ifs, raw_map)){
          AERROR << "fail to parse!" << filename;
          return false;
      }

      if(raw_map.empty()){
          AERROR << "no data in json map!";
          return false;
      }

      // parse header
      Json::Value header = raw_map["header"];
      AINFO << "HDmap Version: " << header["version"].asString() << "  date: " << header["date"].asString();

      // parse roads
      Json::Value raw_map_lanes = raw_map["lane"];
      bool status = RoadsJsonParser::Parse(raw_map_lanes, map->roads);
      if (!status) {
        AERROR << "fail to parse json roads.";
        return false;
      }

      // // parse junctions
      // Json::Value raw_map_junctions = raw_map["junction"];
      // status = JunctionsJsonParser::Parse(raw_map_junctions, map->junctions);
      // if (!status) {
      //   AERROR << "fail to parse json junctions.";
      //   return false;
      // }

      // // parse stop signs
      // Json::Value raw_map_stop_signs = raw_map["stop_sign"];
      // status = StopSignsJsonParser::Parse(raw_map_stop_signs, map->stop_signs);
      // if (!status) {
      //   AERROR << "fail to parse json stop signs.";
      //   return false;
      // }

      // // parse signals
      // Json::Value raw_map_signal = raw_map["signal"];
      // status = SignalsJsonParser::Parse(raw_map_signal, map->signals);
      // if (!status) {
      //   AERROR << "fail to parse json signals.";
      //   return false;
      // }
      
      // parse overlaps
      // Json::Value raw_map_overlaps = raw_map["overlap"];
      // status = OverlapsJsonParser::Parse(raw_map_overlaps, map->overlaps);
      // if (!status) {
      //   AERROR << "fail to parse json overlaps.";
      //   return false;
      // }

      return true;
    }//LoadData
  }
}