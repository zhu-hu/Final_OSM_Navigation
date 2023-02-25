//
// Created by luyifan on 18-7-12.
//

#ifndef STATEMACHINE_TIGGO_MODEL_H
#define STATEMACHINE_TIGGO_MODEL_H

#include <typeinfo>

#include "common/utils/find_path.h"
#include "parameter/Parameter.h"

namespace planning {

class Model : public Parameter {
 public:
  Model(ros::NodeHandle *pnd_) : Parameter() {
    /************************************
            行为参数
    ***********************************/
    double default_emergency_stop_area_width = 2.0;
    double default_emergency_stop_area_height = 5.0;
    double default_emergency_stop_grid_count_threshold = 150;
    double default_emergency_stop_time_count_threshold = 8;
    double default_emergency_speed_down_area_width = 2.4;
    double default_emergency_speed_down_area_height = 6.0;
    double default_emergency_speed_down_grid_count_threshold = 220;
    double default_emergency_speed_down_time_count_threshold = 8;
    double default_protect_area_width = 2.0;
    double default_protect_area_height = 1.0;
    pnd_->param("behavior/emergency_stop_area_width",
                behavior_param_.emergency_stop_area_width,
                default_emergency_stop_area_width);
    pnd_->param("behavior/emergency_stop_area_height",
                behavior_param_.emergency_stop_area_height,
                default_emergency_stop_area_height);
    pnd_->param("behavior/emergency_stop_grid_count_threshold",
                behavior_param_.emergency_stop_grid_count_threshold,
                default_emergency_stop_grid_count_threshold);
    pnd_->param("behavior/emergency_stop_time_count_threshold",
                behavior_param_.emergency_stop_time_count_threshold,
                default_emergency_stop_time_count_threshold);
    pnd_->param("behavior/emergency_speed_down_area_width",
                behavior_param_.emergency_speed_down_area_width,
                default_emergency_speed_down_area_width);
    pnd_->param("behavior/emergency_speed_down_area_height",
                behavior_param_.emergency_speed_down_area_height,
                default_emergency_speed_down_area_height);
    pnd_->param("behavior/emergency_speed_down_grid_count_threshold",
                behavior_param_.emergency_speed_down_grid_count_threshold,
                default_emergency_speed_down_grid_count_threshold);
    pnd_->param("behavior/emergency_speed_down_time_count_threshold",
                behavior_param_.emergency_speed_down_time_count_threshold,
                default_emergency_speed_down_time_count_threshold);
    pnd_->param("behavior/emergency_stop_area_width",
                behavior_param_.protect_area_width, default_protect_area_width);
    pnd_->param("behavior/emergency_stop_area_height",
                behavior_param_.protect_area_height,
                default_protect_area_height);

    double default_lat_safe_distance_static_obstacle = 0.3;
    pnd_->param("behavior/lat_safe_distance_static_obstacle",
                behavior_param_.lat_safe_distance_static_obstacle,
                default_lat_safe_distance_static_obstacle);

    double default_invalid_localization_pos_diff_threshold = 0.2;
    pnd_->param("behavior/invalid_localization_pos_diff_threshold",
                behavior_param_.invalid_localization_pos_diff_threshold,
                default_invalid_localization_pos_diff_threshold);

    double default_stop_interval = 60;
    pnd_->param("behavior/stop_interval", behavior_param_.stop_interval,
                default_stop_interval);

    double default_max_distance_away_from_path = 10.0;
    pnd_->param("behavior/max_distance_away_from_path",
                behavior_param_.max_distance_away_from_path,
                default_max_distance_away_from_path);

    pnd_->param("vehicle/size/width", behavior_param_.vehicle_width, 1.6);
    pnd_->param("vehicle/size/length", behavior_param_.vehicle_length, 2.5);
    pnd_->param("vehicle/size/rear_to_back", behavior_param_.rear_to_back,
                0.35);
    pnd_->param("vehicle/turning_radius", behavior_param_.turning_radius, 6.0);
    pnd_->param("vehicle/size/trailer_length", behavior_param_.trailer_length,
                2.04);

    pnd_->param("behavior/wheel_base", behavior_param_.wheel_base, 1.6);
    pnd_->param("behavior/desired_planning_speed",
                behavior_param_.desired_planning_speed, 2.0);  // m/s
    pnd_->param("behavior/max_planner_time", behavior_param_.max_planner_time,
                100.0);

    pnd_->param("behavior/max_lat_sample_nums",
                behavior_param_.max_lat_sample_nums, 15);
    pnd_->param("behavior/max_lon_sample_nums",
                behavior_param_.max_lon_sample_nums, 7);

    pnd_->param("behavior/smoothed_route_path",
                behavior_param_.smoothed_route_path, true);
    pnd_->param("behavior/smoothed_range", behavior_param_.smoothed_range, 1.0);

    /************************************
            地图参数
    ***********************************/
    double default_utm_origin_x = 355000;
    double default_utm_origin_y = 2700000;
    pnd_->param("GLOBAL_ZERO_X_", map_param_.utm_origin_x,
                default_utm_origin_x);
    pnd_->param("GLOBAL_ZERO_Y_", map_param_.utm_origin_y,
                default_utm_origin_y);

    /************************************
            速度规划参数
    ***********************************/
    double default_desire_speed_lane = 15.0;
    double default_desire_speed_intersection = 8.0;
    double default_desire_speed_intersection_straight = 15.0;
    double default_accelerate_up = 0.5;
    double default_accelerate_down = 0.5;
    double default_maximum_speed = 20.0;
    double default_desire_speed_uturn = 6.0;
    double default_desire_speed_lane_change = 10.0;
    double default_desire_speed_slow_down = 5.0;
    //在直道上行驶的正常速度,单位:m/s
    pnd_->param("speed/desire_speed_lane", speed_gen_param_.desire_speed_lane_,
                default_desire_speed_lane);
    //在路口拐弯的正常速度,单位:m/s
    pnd_->param("speed/desire_speed_intersection",
                speed_gen_param_.desire_speed_intersection_,
                default_desire_speed_intersection);
    //在路口直行的正常速度,单位:m/s
    pnd_->param("speed/desire_speed_intersection_straight",
                speed_gen_param_.desire_speed_intersection_straight_,
                default_desire_speed_intersection_straight);
    //加速加速度单位:m/s2
    pnd_->param("speed/accelerate_up", speed_gen_param_.accelerate_up_,
                default_accelerate_up);
    //减速加速度单位:m/s2
    pnd_->param("speed/accelerate_down", speed_gen_param_.accelerate_down_,
                default_accelerate_down);
    //最大速度
    pnd_->param("speed/maximum_speed", speed_gen_param_.maximum_speed,
                default_maximum_speed);
    // uturn速度
    pnd_->param("speed/desire_speed_uturn",
                speed_gen_param_.desire_speed_uturn_,
                default_desire_speed_uturn);
    // 换道
    pnd_->param("speed/desire_speed_lane_change",
                speed_gen_param_.desire_speed_lane_change,
                default_desire_speed_lane_change);
    //特定路段减速慢行 单位:m/s （室内）
    pnd_->param("speed/desire_speed_slow_down",
                speed_gen_param_.desire_speed_slow_down_,
                default_desire_speed_slow_down);
    speed_gen_param_.desire_speed_lane_ /= 3.6;
    speed_gen_param_.desire_speed_intersection_ /= 3.6;
    speed_gen_param_.desire_speed_intersection_straight_ /= 3.6;
    speed_gen_param_.desire_speed_slow_down_ /= 3.6;
    speed_gen_param_.desire_speed_lane_change /= 3.6;
    speed_gen_param_.desire_speed_uturn_ /= 3.6;
    speed_gen_param_.desire_speed_idling_ = 12 / 3.6;
    speed_gen_param_.desire_low_speed_ = 10 / 3.6;
    speed_gen_param_.desire_speed_change_lane_ = 15 / 3.6;
    speed_gen_param_.maximum_speed /= 3.6;

    /***********************************
            栅格图参数
    ***********************************/
    double default_grid_map_pixel_scale = 10.0;
    double default_grid_map_min_x = -10.0;
    double default_grid_map_max_x = 30.0;
    double default_grid_map_min_y = -10.0;
    double default_grid_map_max_y = 10.0;
    pnd_->param("grid_map/roi_map/pixel_scale", grid_map_param_.pixel_scale,
                default_grid_map_pixel_scale);
    pnd_->param("grid_map/roi_map/min_x", grid_map_param_.min_x,
                default_grid_map_min_x);
    pnd_->param("grid_map/roi_map/max_x", grid_map_param_.max_x,
                default_grid_map_max_x);
    pnd_->param("grid_map/roi_map/min_y", grid_map_param_.min_y,
                default_grid_map_min_y);
    pnd_->param("grid_map/roi_map/max_y", grid_map_param_.max_y,
                default_grid_map_max_y);

    /*************************************
     * OSM地图参数
     * **********************************/
    // OSM地图中需要解析的way的类型
    std::vector<std::string> default_osm_ways_filter{"service"};
    pnd_->param("osm_map/ways_filter", map_param_.osm_ways_filter,
                default_osm_ways_filter);

    // OSM地图中路点之间的间隔距离
    pnd_->param("osm_map/interpolation_max_distance",
                map_param_.osm_interpolation_max_distance, 10.0);

    // OSM地图的文件路径
    std::string default_osm_map_path =
        expand_catkin_ws("/src/osm_parser/data/sjtu_dongqu.osm");
    pnd_->param("osm_map_path", map_param_.osm_map_path, default_osm_map_path);
  }
};
}  // namespace planning

#endif  // STATEMACHINE_TIGGO_MODEL_H
