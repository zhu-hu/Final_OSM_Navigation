//
// Created by luyifan on 18-7-12.
//

#ifndef STATEMACHINE_BEHAVIORPARAM_H
#define STATEMACHINE_BEHAVIORPARAM_H
#include <stdint.h>
#include <string.h>

namespace planning {
class BehaviorParam {
 public:
  BehaviorParam() {}

 public:
  double emergency_stop_area_width;
  double emergency_stop_area_height;
  double emergency_stop_grid_count_threshold;
  double emergency_stop_time_count_threshold;
  double emergency_speed_down_area_width;
  double emergency_speed_down_area_height;
  double emergency_speed_down_grid_count_threshold;
  double emergency_speed_down_time_count_threshold;
  double protect_area_width;
  double protect_area_height;

  double lat_safe_distance_static_obstacle;  //静态障碍物的横向安全距离

  double invalid_localization_pos_diff_threshold;

  double stop_interval;

  double max_distance_away_from_path;

  double vehicle_width;
  double vehicle_length;
  double rear_to_back;
  double turning_radius;
  double trailer_length;
  double wheel_base;

  double desired_planning_speed;
  double max_planner_time;

  // dubins planner里的参数
  int max_lat_sample_nums;
  int max_lon_sample_nums;

  bool smoothed_route_path;
  double smoothed_range;
};
}  // namespace planning

#endif  // STATEMACHINE_BEHAVIORPARAM_H
