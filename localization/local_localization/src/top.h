/**
 *top.h
 *brief:top layer of localization pipeline
 *author:Chen Xiaofeng
 *date:20191028
 **/

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <iostream>

#include "cyber_msgs/Heading.h"
#include "cyber_msgs/LocalizationEstimate.h"
#include "cyber_msgs/VehicleSpeedFeedback.h"
#include "cyber_msgs/VehicleSteerFeedback.h"
#include "ekf/ekf_pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/TimeReference.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "visualization_msgs/Marker.h"

#ifndef TOP_H
#define TOP_H

class Top {
 public:
  Top(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);
  ~Top();

 private:
  bool have_inited_vel_ = false;
  bool have_inited_gps_ = false;
  bool command_no_gps_ = false;
  bool gps_with_problem_ = false;
  bool localization_debug_ = false;
  bool use_steer_ = false;
  int gps_warning_count = 0;
  int gps_error_count = 0;

  double curr_vel_ = 0;
  double curr_yaw_rate_ = 0;
  geometry_msgs::Vector3 angular_velocity_;
  geometry_msgs::Vector3 linear_acceleration_;

  All_EKF_Pose fusion_;

  // parameters for ekf
  double slam_fix_param_ = 0.3;
  double slam_error_param_ = 0.2;
  double imu_err_ = 0.004;       // rad velocity of IMU, unit: rad/s
  double speed_err_ = 0.001;     // unit: m/s
  double gps_err_fix_ = 0.0005;  // from gps, unit: m
  double gps_yaw_err_fix_ = 0.5 * M_PI / 180.0;     // from gps, unit: rad
  double gps_yaw_err_normal_ = 2.0 * M_PI / 180.0;  // from gps, unit: rad
  double slam_err_fix_ = 0.1;                       // slam error, unit: m
  double slam_err_normal_ = 1.0;                    // slam error, unit: m
  double slam_yaw_err_fix_ = 0.5 * M_PI / 180.0;    // from slam, unit: rad
  double slam_yaw_err_normal_ = 5 * M_PI / 180.0;   // from slam, unit: rad

  double t_vel_ = 0.0;
  double t_gps_ = 0.0;
  double t_slam_ = 0.0;
  double t_output_ = 0.0;
  double t_diff_ = 0.0;

  double global_heading_ = 0;
  double cali_x_ = 0;
  double cali_y_ = 0;

  ros::Timer filter_timer_;

  ros::Publisher pub_localization_estimation_;
  ros::Publisher pub_gps_marker_;
  ros::Publisher pub_gps_pose_;
  ros::Publisher pub_slam_marker_;
  ros::Publisher pub_slam_pose_;
  ros::Publisher pub_filter_marker_;
  ros::Publisher pub_filter_pose_;

  ros::Subscriber vel_sub_;
  ros::Subscriber steer_sub_;
  ros::Subscriber sub_imu_;
  ros::Subscriber command_sub_;

  ros::Subscriber sub_global_init_;

  ros::Subscriber sub_reset_flag_;

  ros::Timer timer_;

  ros::Publisher pub_rtk_yaw_;
  ros::Publisher pub_mag_yaw_;
  ros::Publisher pub_heading_yaw_;

  ros::Subscriber sub_mag_;
  ros::Subscriber sub_gps_heading_;

  double init_x_ = 0.0;
  double init_y_ = 0.0;
  double init_theta_ = 0.0;
  bool global_init_flag_ = false;

  // typedef message_filters::sync_policies::ApproximateTime<
  //     sensor_msgs::NavSatFix, cyber_msgs::Heading, cyber_msgs::GPGGA_MSG>
  //     GpsPolicy;
  // message_filters::Subscriber<sensor_msgs::NavSatFix>* sub_gps_fix_;
  // message_filters::Subscriber<cyber_msgs::Heading>* sub_gps_heading_;
  // message_filters::Subscriber<cyber_msgs::GPGGA_MSG>* sub_gps_status_;
  // message_filters::Synchronizer<GpsPolicy>* gps_sync_;

  // typedef message_filters::sync_policies::ApproximateTime<
  //     sensor_msgs::NavSatFix, sensor_msgs::Imu>
  //     SlamPolicy;
  // message_filters::Subscriber<sensor_msgs::NavSatFix>* sub_slam_fix_;
  // message_filters::Subscriber<sensor_msgs::Imu>* sub_slam_heading_;
  // message_filters::Synchronizer<SlamPolicy>* slam_sync_;

  typedef message_filters::sync_policies::ApproximateTime<
      cyber_msgs::VehicleSpeedFeedback, sensor_msgs::Imu>
      EKFPolicy;

  message_filters::Subscriber<cyber_msgs::VehicleSpeedFeedback>*
      sub_speed_feedback_;
  message_filters::Subscriber<sensor_msgs::Imu>* sub_imu_data_;
  message_filters::Synchronizer<EKFPolicy>* ekf_sync_;

  geometry_msgs::PoseArray gps_poses_;
  visualization_msgs::Marker gps_marker_traj_;
  tf::TransformBroadcaster br_gps_;
  geometry_msgs::PoseArray slam_poses_;
  visualization_msgs::Marker slam_marker_traj_;
  tf::TransformBroadcaster br_slam_;
  geometry_msgs::PoseArray filter_poses_;
  visualization_msgs::Marker filter_marker_traj_;
  tf::TransformBroadcaster br_filter_;

  void global_init_callback(
      const cyber_msgs::LocalizationEstimateConstPtr& local_in);

  double get_time_now();
  void ekf_callback(const cyber_msgs::VehicleSpeedFeedbackConstPtr& vel_in,
                    const sensor_msgs::ImuConstPtr& imu_data_in);
  void vel_callback(const cyber_msgs::VehicleSpeedFeedbackConstPtr& vel_in);
  void imu_callback(const geometry_msgs::Vector3StampedConstPtr& imu_in);
  void steer_callback(const cyber_msgs::VehicleSteerFeedbackConstPtr& steer_in);
  void reset_callback(const std_msgs::BoolConstPtr& flag_in);

  void mag_callback(const geometry_msgs::QuaternionStampedConstPtr& mag_in);

  void heading_callback(const cyber_msgs::HeadingConstPtr& heading_in);

  void timer_callback(const ros::TimerEvent&);
};  // class Top

#endif
