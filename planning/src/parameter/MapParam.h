#ifndef STATEMACHINE_MAPPARAM_H
#define STATEMACHINE_MAPPARAM_H

namespace planning {
class MapParam {
 public:
  MapParam() {}

 public:
  double utm_origin_x;
  double utm_origin_y;

  std::vector<std::string> osm_ways_filter;
  double osm_interpolation_max_distance;
  std::string osm_map_path;
};
}  // namespace planning

#endif  // STATEMACHINE_MAPPARAM_H