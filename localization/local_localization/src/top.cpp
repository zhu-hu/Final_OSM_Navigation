/**
 *top.h
 *brief:top layer of localization pipeline
 *author:Chen Xiaofeng
 *date:20191028
 **/

#include "top.h"

Top::Top(ros::NodeHandle &nh, ros::NodeHandle &nh_priv) {
  nh_priv.param<bool>("use_steer", use_steer_, false);
  nh_priv.param<bool>("localization_debug", localization_debug_, false);
  nh_priv.param<double>("speed_err", speed_err_, 0.001);
  nh_priv.param<double>("imu_err", imu_err_, 0.004);
  nh_priv.param<double>("gps_err_fix", gps_err_fix_, 0.0005);
  nh_priv.param<double>("gps_yaw_err_fix", gps_yaw_err_fix_,
                        0.5 * M_PI / 180.0);
  nh_priv.param<double>("gps_yaw_err_normal", gps_yaw_err_normal_,
                        2.0 * M_PI / 180.0);
  nh_priv.param<double>("slam_err_fix", slam_err_fix_, 0.2);
  nh_priv.param<double>("slam_err_normal", slam_err_normal_, 1.0);
  nh_priv.param<double>("slam_yaw_err_fix", slam_yaw_err_fix_,
                        2 * M_PI / 180.0);
  nh_priv.param<double>("slam_yaw_err_normal", slam_yaw_err_normal_,
                        5 * M_PI / 180.0);
  nh_priv.param<double>("slam_fix_param", slam_fix_param_, 0.3);
  nh_priv.param<double>("slam_error_param", slam_error_param_, 0.2);
  nh_priv.param<double>("cali_x_", cali_x_, 0.46);
  nh_priv.param<double>("cali_y_", cali_y_, -0.32);

  pub_localization_estimation_ =
      nh.advertise<cyber_msgs::LocalizationEstimate>("/local/localization", 1);

  if (localization_debug_) {
    pub_gps_marker_ =
        nh.advertise<visualization_msgs::Marker>("/gps_markers", 2);
    pub_gps_pose_ = nh.advertise<geometry_msgs::PoseArray>("/gps_poses", 2);
    pub_slam_marker_ =
        nh.advertise<visualization_msgs::Marker>("/slam_markers", 2);
    pub_slam_pose_ = nh.advertise<geometry_msgs::PoseArray>("/slam_poses", 2);
    pub_filter_marker_ =
        nh.advertise<visualization_msgs::Marker>("/filter_markers", 2);
    pub_filter_pose_ =
        nh.advertise<geometry_msgs::PoseArray>("/filter_poses", 2);
  }

  sub_speed_feedback_ =
      new message_filters::Subscriber<cyber_msgs::VehicleSpeedFeedback>(
          nh, "/e100/speed_feedback", 10);
  sub_imu_data_ =
      new message_filters::Subscriber<sensor_msgs::Imu>(nh, "/imu/data", 10);

  ekf_sync_ = new message_filters::Synchronizer<EKFPolicy>(
      EKFPolicy(10), *sub_speed_feedback_, *sub_imu_data_);
  ekf_sync_->registerCallback(boost::bind(&Top::ekf_callback, this, _1, _2));

  sub_global_init_ = nh.subscribe("/localization/estimation", 100,
                                  &Top::global_init_callback, this);

  sub_reset_flag_ =
      nh.subscribe("/global_init_reset", 10, &Top::reset_callback, this);

  timer_ = nh.createTimer(ros::Duration(2), &Top::timer_callback, this);

  pub_rtk_yaw_ = nh.advertise<std_msgs::Float64>("/rtk_yaw", 5);

  pub_mag_yaw_ = nh.advertise<std_msgs::Float64>("/imu_yaw", 5);

  pub_heading_yaw_ = nh.advertise<std_msgs::Float64>("/heading_yaw", 5);

  sub_mag_ = nh.subscribe("/filter/quaternion", 10, &Top::mag_callback, this);

  sub_gps_heading_ =
      nh.subscribe("/strong/heading", 10, &Top::heading_callback, this);
  ROS_INFO("Localization Pipeline begin!");

  //   double test_lon = 121.43639320451585;
  //   double test_lat = 31.03063158746496;
  //   char zone;
  //   double test_x = 0;
  //   double test_y = 0;
  //   LLtoUTM(test_lat, test_lon, test_y, test_x, &zone);
  //   std::cout << "testx:" << test_x << " test y:" << test_y << std::endl;
}

Top::~Top() {
  delete sub_speed_feedback_;
  delete sub_imu_data_;
  delete ekf_sync_;
}

void Top::global_init_callback(
    const cyber_msgs::LocalizationEstimateConstPtr &local_in) {
  if (global_init_flag_ == false) {
    init_x_ = local_in->pose.position.x;
    init_y_ = local_in->pose.position.y;
    init_theta_ = tf::getYaw(local_in->pose.orientation);
    global_init_flag_ = true;
    fusion_.Reset(init_x_, init_y_, init_theta_);
  }
  std_msgs::Float64 msg;
  msg.data = tf::getYaw(local_in->pose.orientation) * 180.0 / M_PI;
  pub_rtk_yaw_.publish(msg);
}

void Top::reset_callback(const std_msgs::BoolConstPtr &flag_in) {
  if (flag_in->data == true) {
    global_init_flag_ = false;
  }
}

void Top::mag_callback(const geometry_msgs::QuaternionStampedConstPtr &mag_in) {
  std_msgs::Float64 msg;
  msg.data = tf::getYaw(mag_in->quaternion) * 180.0 / M_PI;
  pub_mag_yaw_.publish(msg);
}

void Top::heading_callback(const cyber_msgs::HeadingConstPtr &heading_in) {
  std_msgs::Float64 msg;
  msg.data = heading_in->data * 180.0 / M_PI;
  pub_heading_yaw_.publish(msg);
}

double Top::get_time_now() { return ros::Time::now().toSec() - t_diff_; }

// void Top::command_callback(const std_msgs::Int8ConstPtr &bool_in)
// {
//     if(bool_in->data > 0) command_no_gps_ = true;
//     else command_no_gps_ = false;
// }

void Top::ekf_callback(const cyber_msgs::VehicleSpeedFeedbackConstPtr &vel_in,
                       const sensor_msgs::ImuConstPtr &imu_data_in) {
  ROS_INFO("data in!");
  if (global_init_flag_ == false) {
    ROS_INFO("wait_for_init!!!");
    return;
  }

  curr_vel_ = (double)(vel_in->speed_cmps) / 100.0;

  curr_yaw_rate_ = imu_data_in->angular_velocity.z;
  if (curr_yaw_rate_ < 0.00001 && curr_yaw_rate_ > -0.00001)
    curr_yaw_rate_ = 0.00001;

  Eigen::Vector2d Z_vel;
  Eigen::Matrix2d R_vel;
  Z_vel << curr_vel_, curr_yaw_rate_;
  R_vel << pow(speed_err_, 2), 0, 0, pow(imu_err_, 2);

  t_vel_ = ros::Time::now().toSec();
  if (fabs(curr_vel_) > 0.01)
    fusion_.velStateUpdate(Z_vel, R_vel, t_vel_);
  else
    fusion_.timeUpdate(t_vel_);

  auto X_est = fusion_.readX(0.0, false);

  cyber_msgs::LocalizationEstimate msg;
  msg.pose.position.x = X_est(0);
  msg.pose.position.y = X_est(1);
  msg.pose.position.z = 0;
  auto tf_q = tf::createQuaternionFromYaw(X_est(2));
  msg.pose.orientation.x = tf_q.getX();
  msg.pose.orientation.y = tf_q.getY();
  msg.pose.orientation.z = tf_q.getZ();
  msg.pose.orientation.w = tf_q.getW();

  pub_localization_estimation_.publish(msg);
  //   std_msgs::Float64 msg2;
  //   msg2.data = X_est(2);
  //   pub_mag_yaw_.publish(msg2);
}

void Top::timer_callback(const ros::TimerEvent &) { global_init_flag_ = false; }

// void Top::vel_callback(const cyber_msgs::VehicleSpeedFeedbackConstPtr
// &vel_in) {
//   if (!have_inited_gps_) {
//     ROS_INFO("wait for GPS input!");
//     return;
//   }

//   have_inited_vel_ = true;

//   curr_vel_ = (double)(vel_in->speed_cmps) / 100.0;

//   Eigen::Vector2d Z_vel;
//   Eigen::Matrix2d R_vel;
//   Z_vel << curr_vel_, curr_yaw_rate_;
//   R_vel << pow(speed_err_, 2), 0, 0, pow(imu_err_, 2);

//   t_vel_ = get_time_now();
//   if (fabs(curr_vel_) > 0.01)
//     fusion_.velStateUpdate(Z_vel, R_vel, t_vel_);
//   else
//     fusion_.timeUpdate(t_vel_);
// }

// void Top::imu_callback(const geometry_msgs::Vector3StampedConstPtr &imu_in) {
//   curr_yaw_rate_ = imu_in->vector.z;
//   if (curr_yaw_rate_ < 0.00001 && curr_yaw_rate_ > -0.00001)
//     curr_yaw_rate_ = 0.00001;
//   angular_velocity_.x = 0;
//   angular_velocity_.y = 0;
//   angular_velocity_.z = curr_yaw_rate_;
//   // ROS_INFO("yaw rate from imu: %f", curr_yaw_rate_);
// }

// void Top::steer_callback(
//     const cyber_msgs::VehicleSteerFeedbackConstPtr &steer_in) {
//   double angle = -steer_in->steer_0p1d / 10.0 * M_PI / 180.0 /
//                  16.0;  // 16.0 是方向盘转角和前轮转角的比例
//   double delta_theta = (curr_vel_ * 0.01) * tan(angle) / 1.6;
//   curr_yaw_rate_ = delta_theta / 0.01;
//   if (curr_yaw_rate_ < 0.00001 && curr_yaw_rate_ > -0.00001)
//     curr_yaw_rate_ = 0.00001;
//   angular_velocity_.x = 0;
//   angular_velocity_.y = 0;
//   angular_velocity_.z = curr_yaw_rate_;
//   // ROS_INFO("yaw rate from steer: %f", curr_yaw_rate_);
// }