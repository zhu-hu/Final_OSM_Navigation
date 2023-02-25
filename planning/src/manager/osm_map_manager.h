#ifndef OSM_MAP_MANAGER_H
#define OSM_MAP_MANAGER_H
#include <ros/ros.h>

#include "osm_parser/src/osm_parser.h"
#include "parameter/tiggo_model.h"
namespace planning {
class OsmMapManager {
 public:
  OsmMapManager(ros::NodeHandle *nh, Parameter *param);

  ~OsmMapManager();

  osm_parser::Parser *osm_map() { return osm_map_; }

 private:
  osm_parser::Parser *osm_map_;

  Parameter *param_;

  ros::NodeHandle *nh_;
};
}  // namespace planning

#endif