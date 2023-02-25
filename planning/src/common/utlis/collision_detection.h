//
// Created by wl on 2019/11/7.
//

#ifndef PLANNING_COLLISION_DETECTION_H
#define PLANNING_COLLISION_DETECTION_H

#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "../struct/position.h"
#include "../struct/prediction_obstacles.h"
#include "../struct/trajectory.h"
#include "common/params/vehicle_model.h"
#include "common/utils/ros_transform.h"
#include "cyber_msgs/LocalizationEstimate.h"
#include "map/hdmap/src/math/Polygon.h"
#include "parameter/tiggo_model.h"
#include "tf/transform_listener.h"

using namespace common::math;

namespace planning {
bool collision_detect(Trajectory *trajectory,
                      const std::vector<PredictionObstacle> &obstacles);
bool collision_detect(Trajectory *trajectory,
                      const std::vector<PredictionObstacle> &obstacles,
                      const cv::Mat &grid_map, const Parameter *param);
bool collision_detect_for_trajectories(
    const std::vector<Trajectory> *trajectories_,
    const std::vector<PredictionObstacle> &obstacles_,
    const cv::Mat &grid_map_);
bool collision_detect_for_sidewalk(
    const std::vector<PredictionObstacle> &obstacles,
    const Polygon2d &sidewalk_detection_area);
bool collision_detect_for_cautious_zone(
    const std::vector<PredictionObstacle> &obstacles,
    const Polygon2d &cautious_zone_area, const cv::Mat &grid_map,
    const Parameter *param);
bool collision_detect_for_lane_change(
    const std::vector<PredictionObstacle> &obstacles,
    const Polygon2d &lane_change_detection_area,
    const double lane_change_ignore_oncoming_vehicles_velocity_threshold);
bool collision_detect_for_unprotected_turn(
    const std::vector<PredictionObstacle> &obstacles,
    const Polygon2d &detection_area, const double &ego_yaw);
bool collision_detect_for_rrt(Trajectory *trajectory,
                              const std::vector<PredictionObstacle> &obstacles);
bool global_to_local(const Vec2d &global_pt, Vec2d &local_pt);
double normalize_angle(double angle);
}  // namespace planning

#endif  // PLANNING_COLLISION_DETECTION_H
