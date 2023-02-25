//
// Created by Chen Xiaofeng on 19-11-1.
//

#ifndef STATEMACHINE_TRAJECTORY_H
#define STATEMACHINE_TRAJECTORY_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <string>
#include <vector>

#include "common/utils/bezier.h"
#include "common/utils/log.h"
#include "common/utils/tool.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "hdmap/src/Impl.h"
#include "prediction_obstacles.h"
#include "trajectory_point.h"
namespace planning {

class Trajectory {
 private:
  std::vector<TrajectoryPoint> points_;
  double cost_;
  double lateral_distance_static_obstacle_;
  double longitude_distance_static_obstacle_;
  double lateral_distance_dynamic_obstacle_;
  double longitude_distance_dynamic_obstacle_;
  double lateral_distance_pedestrian_;
  double longitude_distance_pedestrian_;
  double lateral_distance_cone_;
  double longitude_distance_cone_;
  std::vector<std::pair<double, double>> obstacles_;
  bool drivable_;
  double lat_offset_to_adc_;
  double expect_speed_;
  PredictionObstacle front_dynamic_obstacle_;
  double front_dynamic_obstacle_speed_;  //前方动态障碍物速度

  void UpdateDistance2Obstacle();
  void UpdateDrivable();
  void UpdateCost();

 public:
  Trajectory();
  ~Trajectory();
  void UpdateStatus();
  bool CreateTrajectory(const std::vector<TrajectoryPoint> &trajectory,
                        double expect_speed);
  bool CreateTrajectory(const std::vector<TrajectoryPoint> &trajectory_points);
  bool CreateTrajectory(const std::vector<TrajectoryPoint> &trajectory_points,
                        int index);
  inline void SetCost(double cost) { cost_ = cost; }
  inline void set_front_dynamic_obstacle(PredictionObstacle obs) {
    front_dynamic_obstacle_ = obs;
  }
  inline void set_front_dynamic_obstacle_speed(double obs_speed) {
    front_dynamic_obstacle_speed_ = obs_speed;
  }
  inline double front_dynamic_obstacle_speed() {
    return front_dynamic_obstacle_speed_;
  }
  inline const PredictionObstacle front_dynamic_obstacle() {
    return front_dynamic_obstacle_;
  }
  inline void set_lateral_offset_to_adc(const double lat_offset_to_adc) {
    lat_offset_to_adc_ = lat_offset_to_adc;
  }
  inline const double lateral_offset_to_adc() { return lat_offset_to_adc_; }
  inline bool GetDrivable() const { return drivable_; }
  inline double GetCost() const { return cost_; }
  inline const std::vector<TrajectoryPoint> &points() const { return points_; }
  inline std::vector<TrajectoryPoint> &mutable_points() { return points_; }
  const inline double lateral_distance_to_static_obstacle() {
    return lateral_distance_static_obstacle_;
  }
  const inline double longitude_distance_to_static_obstacles() const {
    return longitude_distance_static_obstacle_;
  }
  const inline double lateral_distance_to_dynamic_obstacle() {
    return lateral_distance_dynamic_obstacle_;
  }
  const inline double longitude_distance_to_dynamic_obstacles() const {
    return longitude_distance_dynamic_obstacle_;
  }
  const inline double lateral_distance_to_pedestrian() {
    return lateral_distance_pedestrian_;
  }
  const inline double longitude_distance_to_pedestrian() const {
    return longitude_distance_pedestrian_;
  }
  const inline double lateral_distance_to_cone() {
    return lateral_distance_cone_;
  }
  const inline double longitude_distance_to_cone() const {
    return longitude_distance_cone_;
  }
  inline void set_lateral_distance_to_static_obstacle(const double distance) {
    lateral_distance_static_obstacle_ = distance;
  }
  inline void set_longitude_distance_to_static_obstacle(const double distance) {
    longitude_distance_static_obstacle_ = distance;
  }
  inline void set_lateral_distance_to_dynamic_obstacle(const double distance) {
    lateral_distance_dynamic_obstacle_ = distance;
  }
  inline void set_longitude_distance_to_dynamic_obstacle(
      const double distance) {
    longitude_distance_dynamic_obstacle_ = distance;
  }
  inline void set_lateral_distance_to_pedestrian(const double distance) {
    lateral_distance_pedestrian_ = distance;
  }
  inline void set_longitude_distance_to_pedestrian(const double distance) {
    longitude_distance_pedestrian_ = distance;
  }
  inline void set_lateral_distance_to_cone(const double distance) {
    lateral_distance_cone_ = distance;
  }
  inline void set_longitude_distance_to_cone(const double distance) {
    longitude_distance_cone_ = distance;
  }
  double GetMaxNudgeLength();

  inline double expect_speed() { return expect_speed_; }
  double last_frame_id_ = 0;
  bool last_frame_id_exist = false;
  double _cv = 0.0;
  double _cd = 0.0;
  double _cf = 0.0;
  double _jerk = 0;
  double _l_offset = 0;
  double _same_trajectory_count = 1;
  double start_velocity = 0.0;
  double _current_car_l = 0.0;
  double _current_road_width = 0.0;
  int _id;
  int _total_num;
  bool is_collided = false;
  double _lane_heading = 0;
  //横向采样的数值，越大越偏向两边
  int sample_index_;
};
}  // namespace planning

#endif  // STATEMACHINE_TRAJECTORY_H
