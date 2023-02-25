//
// Created by luyifan on 18-3-27.
//

#ifndef STATEMACHINE_SELFSTATEMANAGER_H
#define STATEMACHINE_SELFSTATEMANAGER_H

#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <iostream>
#include <queue>

#include "../parameter/Parameter.h"
#include "common/struct/reference_line.h"
#include "cyber_msgs/LocalizationEstimate.h"
#include "cyber_msgs/VehicleState.h"
#include "tool.h"
// #include "map_manager.h"

namespace planning {

typedef enum LocalizeMode { GPS, LASER_COUNTRYROAD, LASER_TUNNEL } LocalizeMode;

class SelfStateManager {
 public:
  SelfStateManager(ros::NodeHandle *nh, Parameter *param);
  ~SelfStateManager();
  inline const cyber_msgs::LocalizationEstimate GetLocalizationEstimation()
      const {
    return localization_estimation_;
  }
  void IsLocalizationValid(
      const cyber_msgs::LocalizationEstimate &localization_estimation);
  inline double GetHeading() const { return heading_; }
  inline double GetVelocity() const { return velocity_; }
  // bool GetLocalizationValidity() const;
  bool IgnoreObstacle() const { return ignore_obstacle_; }
  bool GetLocalizationValidity() const { return is_localization_valid_; }
  void SetLocalizeMode(LocalizeMode mode);
  void SetReferenceLine(ReferenceLine *reference_line) {
    reference_line_ = reference_line;
  }
  void PublishEmergencyMode(const int &emergency_mode);
  inline const bool GetEmergencyCommand() { return _emergency_command; }

 private:
  cyber_msgs::LocalizationEstimate localization_estimation_;
  cyber_msgs::LocalizationEstimate last_frame_localization_estimation_;
  double heading_;
  double velocity_;

  bool is_initialized_ = false;
  bool is_localization_valid_ = false;
  double invalid_localization_pos_diff_threshold_;

  bool is_auto_state_ = true;

  bool _emergency_command = false;  // true时紧急刹停

  bool ignore_obstacle_ = false;  //忽略障碍物

  bool handware_state_ = true;  // 硬件状态

  ros::NodeHandle *nh_;
  tf::TransformBroadcaster *br_;
  Parameter *param_;
  ReferenceLine *reference_line_ = nullptr;
  ros::Subscriber sub_localization_estimation_;
  ros::Subscriber sub_emergency_command_;
  ros::Subscriber sub_cancell_task_command_;
  ros::Subscriber sub_start_task_command_;
  ros::Subscriber sub_vehicle_state_;
  ros::Subscriber sub_ignore_obstacle_command_;
  ros::Subscriber sub_handware_state_;
  ros::Publisher pub_localize_mode_;
  ros::Publisher pub_velocity_;
  ros::Publisher pub_emergency_mode_;

  void LocalizationEstimationCallback(
      cyber_msgs::LocalizationEstimate localization_estimation);
  void IgnoreObstacleCommandCallback(const std_msgs::Bool &ignore_obstacle);
  void CancellTaskCommandCallback(const std_msgs::Int32 &cancell_task);
  void StartTaskCommandCallback(const std_msgs::Bool &start_task);
  void EmergencyCommandCallback(const std_msgs::Bool &emergency_command);
  void HandwareStateCallback(const std_msgs::Bool &handware_status);
  void VehicleStateCallback(const cyber_msgs::VehicleState &vehicle_state);
};
}  // namespace planning

#endif  // STATEMACHINE_SELFSTATEMANAGER_H
