//
// Created by Chen Xiaofeng on 19-10-20.
//

#ifndef HDMAP_JSON_ADAPTER_H
#define HDMAP_JSON_ADAPTER_H

#include <jsoncpp/json/json.h> 
#include <fstream>
#include "roads_json_parser.h"
#include "junctions_json_parser.h"
#include "overlaps_json_parser.h"
#include "stop_signs_json_parser.h"
#include "signals_json_parser.h"

namespace hdmap{
  namespace adapter{
    class JsonAdapter {
    public:
      static bool LoadData(const std::string& filename, Map* map);
    };
  }
}

#endif //HDMAP_JSON_ADAPTER_H
