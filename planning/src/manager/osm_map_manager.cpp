#include "osm_map_manager.h"

namespace planning {
OsmMapManager::OsmMapManager(ros::NodeHandle *nh, Parameter *param)
    : param_(param), nh_(nh) {
  osm_map_ = new osm_parser::Parser;
  std::string osm_map_path = param_->map_param_.osm_map_path;
  osm_map_->setNewMap(osm_map_path);
  std::vector<std::string> ways_filter = param_->map_param_.osm_ways_filter;
  osm_map_->setTypeOfWays(ways_filter);
  double interpolation_max_distance =
      param->map_param_.osm_interpolation_max_distance;
  osm_map_->setInterpolationMaxDistance(interpolation_max_distance);

  osm_map_->parse();
}

OsmMapManager::~OsmMapManager() { delete osm_map_; }
}  // namespace planning
