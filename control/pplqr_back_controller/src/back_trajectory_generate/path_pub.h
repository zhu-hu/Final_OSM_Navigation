#pragma once

#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt64.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <limits>
#include <list>
#include <string>

#include "cyber_msgs/LinkAngle.h"
#include "cyber_msgs/LocalTrajList.h"
#include "cyber_msgs/LocalizationEstimate.h"
#include "visualization.h"

namespace path_pub {

typedef std::vector<std::pair<double, double>> Point2DS;

struct NowTrajectory {
  cyber_msgs::LocalTrajList *trajectory = nullptr;
  bool mode = false;
  std::size_t nearest_index = 0;
  std::size_t stop_index = std::numeric_limits<std::size_t>::max();
};

// struct LocalTrajectories{
//     std::vector<cyber_msgs::LocalTrajList*> trajectories;
// };

class PathPub {
 public:
  PathPub(ros::NodeHandle *nh);
  ~PathPub();

  void InitCommonTrajectory(const std::string &common_trajectory_path);
  void SetTrajectory(std::list<cyber_msgs::LocalTrajList> *trajectories_in);
  void ResetTrajectory();

 private:
  ros::NodeHandle *nh_;

  ros::Publisher pub_trajectory_;
  ros::Publisher pub_finished_;
  ros::Publisher pub_stop_index_;
  ros::Subscriber sub_start_;
  ros::Subscriber sub_loaclization_;

  ros::Publisher pub_rviz_global_path_;
  ros::Publisher pub_rviz_local_map_;
  ros::Publisher pub_rviz_pose_;

  ros::Publisher pub_rviz_tractor_real_path_;
  ros::Publisher pub_rviz_trailer_real_path_;

  ros::Publisher pub_trailer_real_pose_;

  ros::Subscriber sub_reset_real_path_;

  //发布所有轨迹都完成控制的消息
  ros::Publisher pub_all_trajectories_done_;
  //订阅某一段轨迹完成的消息，便于发布下一段轨迹
  ros::Subscriber sub_one_trajectory_finished_;

  nav_msgs::Path tractor_path_;
  nav_msgs::Path trailer_path_;

  typedef message_filters::sync_policies::ApproximateTime<
      cyber_msgs::LocalizationEstimate, cyber_msgs::LinkAngle>
      MotionPolicy;
  message_filters::Subscriber<cyber_msgs::LocalizationEstimate>
      *sub_localization_;
  message_filters::Subscriber<cyber_msgs::LinkAngle> *sub_link_angle_;
  message_filters::Synchronizer<MotionPolicy> *motion_sync_;

  std::list<cyber_msgs::LocalTrajList> common_trajectory_;
  NowTrajectory now_trajectory_;

  std::string package_path_;
  char state_;
  bool bplan_;
  double finish_time_;

  bool use_laser_result_ = false;

  bool localization_flag_ = false;

  bool show_real_path_ = false;

  geometry_msgs::Pose vehicle_pose_;

  void StartCallback(const std_msgs::Int8ConstPtr &start_in);
  void OneTrajectoryFinishedCallback(const std_msgs::Int8ConstPtr &msg);
  void LocalizationCallback(
      const cyber_msgs::LocalizationEstimateConstPtr &localization_in);
  void MotionCallback(
      const cyber_msgs::LocalizationEstimateConstPtr &localization_in,
      const cyber_msgs::LinkAngleConstPtr &link_angle_in);
  void ResetRealPathCallback(const std_msgs::BoolConstPtr &msg);
  double NormalizeAngle(const double angle);
};
}  // namespace path_pub
