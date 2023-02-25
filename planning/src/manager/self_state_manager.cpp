//
// Created by luyifan on 18-3-27.
//

#include "manager/self_state_manager.h"

namespace planning {
SelfStateManager::SelfStateManager(ros::NodeHandle *nh, Parameter *param) {
  nh_ = nh;
  param_ = param;
  invalid_localization_pos_diff_threshold_ =
      param_->behavior_param_.invalid_localization_pos_diff_threshold;
  sub_localization_estimation_ =
      nh_->subscribe("/localization/estimation", 1,
                     &SelfStateManager::LocalizationEstimationCallback, this);
  pub_localize_mode_ = nh_->advertise<std_msgs::Int8>("/localize_mode", 1);
  pub_velocity_ =
      nh_->advertise<std_msgs::Float32>("/visualization/velocity", 10);
  pub_emergency_mode_ =
      nh_->advertise<std_msgs::Int32>("/planning/emergency_byweb", 1);
  sub_ignore_obstacle_command_ =
      nh_->subscribe("/planning/ignore_obstacle", 1,
                     &SelfStateManager::IgnoreObstacleCommandCallback, this);
  sub_cancell_task_command_ =
      nh_->subscribe("/planning/task_id/cancell", 1,
                     &SelfStateManager::CancellTaskCommandCallback, this);
  sub_start_task_command_ =
      nh_->subscribe("planning/task_id/start", 1,
                     &SelfStateManager::StartTaskCommandCallback, this);
  sub_emergency_command_ = nh_->subscribe(
      "/web/emergency", 1, &SelfStateManager::EmergencyCommandCallback, this);
  sub_vehicle_state_ = nh_->subscribe(
      "/e100/vehicle_state", 5, &SelfStateManager::VehicleStateCallback, this);
  sub_handware_state_ = nh_->subscribe(
      "/handware_state", 5, &SelfStateManager::HandwareStateCallback, this);
  br_ = new tf::TransformBroadcaster();

  localization_estimation_.pose.position.x = MY_INF;
  localization_estimation_.pose.position.y = MY_INF;

  is_initialized_ = false;
}

SelfStateManager::~SelfStateManager() { delete br_; }

void SelfStateManager::LocalizationEstimationCallback(
    cyber_msgs::LocalizationEstimate localization_estimation) {
  // if (!is_initialized_)
  // {
  //     AINFO << "no init" << std::endl;
  //     if (localization_estimation.status != 0)
  //     {
  //         last_frame_localization_estimation_.pose =
  //         localization_estimation.pose;
  //         last_frame_localization_estimation_.velocity =
  //         localization_estimation.velocity;

  //         is_initialized_ = true;
  //         is_localization_valid_ = true;
  //     }
  // }
  // else
  // {
  //     if (reference_line_ != nullptr)
  //     {
  //         IsLocalizationValid(localization_estimation);

  //         if (is_localization_valid_)
  //         {
  //             last_frame_localization_estimation_.pose =
  //             localization_estimation.pose;
  //             last_frame_localization_estimation_.velocity =
  //             localization_estimation.velocity;
  //         }
  //     }
  //     else
  //     {
  //         last_frame_localization_estimation_.pose =
  //         localization_estimation.pose;
  //         last_frame_localization_estimation_.velocity =
  //         localization_estimation.velocity;
  //     }
  // }

  if (!is_initialized_) {
    AINFO << "no init" << std::endl;
    if (localization_estimation.status != 0) {
      last_frame_localization_estimation_.pose = localization_estimation.pose;
      last_frame_localization_estimation_.velocity =
          localization_estimation.velocity;

      is_initialized_ = true;
      is_localization_valid_ = true;
    }
  } else {
    IsLocalizationValid(localization_estimation);

    // if (is_localization_valid_)
    // {
    //     last_frame_localization_estimation_.pose =
    //     localization_estimation.pose;
    //     last_frame_localization_estimation_.velocity =
    //     localization_estimation.velocity;
    // }

    if (is_auto_state_) {
      IsLocalizationValid(localization_estimation);
      if (is_localization_valid_) {
        last_frame_localization_estimation_.pose = localization_estimation.pose;
        last_frame_localization_estimation_.velocity =
            localization_estimation.velocity;
      }
    } else if (!is_auto_state_) {
      is_localization_valid_ = true;
      last_frame_localization_estimation_.pose = localization_estimation.pose;
      last_frame_localization_estimation_.velocity =
          localization_estimation.velocity;
    }
  }

  if (localization_estimation.status != 0) {
    localization_estimation_ = localization_estimation;
  } else {
    localization_estimation_.status = 0;
  }

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(localization_estimation_.pose.position.x,
                                  localization_estimation_.pose.position.y,
                                  0.0));
  tf::Quaternion q;
  q.setX(localization_estimation_.pose.orientation.x);
  q.setY(localization_estimation_.pose.orientation.y);
  q.setZ(localization_estimation_.pose.orientation.z);
  q.setW(localization_estimation_.pose.orientation.w);
  transform.setRotation(q);
  double stamp_error = abs(ros::Time::now().toSec() -
                           localization_estimation_.header.stamp.toSec());
  if (stamp_error > 2)
    br_->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                            "world", "base_link"));
  else {
    br_->sendTransform(
        tf::StampedTransform(transform, localization_estimation_.header.stamp,
                             "world", "base_link"));
  }

  common::PointENU localization_point;
  localization_point.x = localization_estimation_.pose.position.x;
  localization_point.y = localization_estimation_.pose.position.y;
  localization_point.z = localization_estimation_.pose.position.z;
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(localization_estimation_.pose.orientation.x,
                               localization_estimation_.pose.orientation.y,
                               localization_estimation_.pose.orientation.z,
                               localization_estimation_.pose.orientation.w))
      .getRPY(roll, pitch, yaw);
  heading_ = yaw;
  velocity_ = localization_estimation_.velocity.linear.x;

  // rviz显示车速
  std_msgs::Float32 v_vel;
  v_vel.data = (float)(velocity_ * 3.6);
  pub_velocity_.publish(v_vel);
}

void SelfStateManager::IgnoreObstacleCommandCallback(
    const std_msgs::Bool &ignore_obstacle) {
  ignore_obstacle_ = ignore_obstacle.data;
}

void SelfStateManager::CancellTaskCommandCallback(
    const std_msgs::Int32 &cancell_task) {
  _emergency_command = true;
}

void SelfStateManager::StartTaskCommandCallback(
    const std_msgs::Bool &start_task) {
  _emergency_command = start_task.data;
}

void SelfStateManager::EmergencyCommandCallback(
    const std_msgs::Bool &emergency_command) {
  _emergency_command = emergency_command.data;
}

void SelfStateManager::VehicleStateCallback(
    const cyber_msgs::VehicleState &vehicle_state_) {
  is_auto_state_ = vehicle_state_.beAutoState;
}

void SelfStateManager::HandwareStateCallback(
    const std_msgs::Bool &handware_state) {
  handware_state_ = handware_state.data;
  _emergency_command = handware_state_;
}

void SelfStateManager::PublishEmergencyMode(const int &emergency_mode) {
  std_msgs::Int32 emergency_mode_;
  emergency_mode_.data = emergency_mode;
  pub_emergency_mode_.publish(emergency_mode_);
}

void SelfStateManager::SetLocalizeMode(LocalizeMode mode) {
  std_msgs::Int8 index;
  switch (mode) {
    case GPS:
      index.data = 0;
      break;
    case LASER_COUNTRYROAD:
      index.data = 1;
      break;
    case LASER_TUNNEL:
      index.data = 2;
      break;
  }
  pub_localize_mode_.publish(index);
}

void SelfStateManager::IsLocalizationValid(
    const cyber_msgs::LocalizationEstimate &localization_estimation) {
  double last_frame_pos_x = last_frame_localization_estimation_.pose.position.x;
  double last_frame_pos_y = last_frame_localization_estimation_.pose.position.y;
  double last_frame_adc_heading =
      tf::getYaw(last_frame_localization_estimation_.pose.orientation);
  double cur_frame_pos_x = localization_estimation.pose.position.x;
  double cur_frame_pos_y = localization_estimation.pose.position.y;
  double cur_frame_adc_heading =
      tf::getYaw(localization_estimation.pose.orientation);

  double x_diff = cur_frame_pos_x - last_frame_pos_x;
  double y_diff = cur_frame_pos_y - last_frame_pos_y;
  double l_diff =
      std::abs(sin(atan2(y_diff, x_diff) - last_frame_adc_heading)) *
      sqrt(pow(y_diff, 2) + pow(x_diff, 2));

  // double l_diff = std::abs(tan(last_frame_adc_heading) * (cur_frame_pos_x -
  // last_frame_pos_x) + last_frame_pos_y - cur_frame_pos_y)
  //                 / sqrt(1 + pow(tan(last_frame_adc_heading), 2));

  // AINFO << "l diff: " << l_diff;

  is_localization_valid_ = l_diff < invalid_localization_pos_diff_threshold_;
}

// void SelfStateManager::IsLocalizationValid(const
// cyber_msgs::LocalizationEstimate& localization_estimation)
// {
//     double last_frame_utm_x =
//     last_frame_localization_estimation_.pose.position.x; double
//     last_frame_utm_y = last_frame_localization_estimation_.pose.position.y;
//     double last_frame_frenet_s, last_frame_frenet_l;
//     double last_frame_adc_heading =
//     tf::getYaw(last_frame_localization_estimation_.pose.orientation); double
//     cur_frame_utm_x = localization_estimation.pose.position.x; double
//     cur_frame_utm_y = localization_estimation.pose.position.y; double
//     cur_frame_frenet_s, cur_frame_frenet_l; double cur_frame_adc_heading =
//     tf::getYaw(localization_estimation.pose.orientation); AINFO << "last
//     frame x: " << last_frame_utm_x << "  cur frame x: " << cur_frame_utm_x;
//     AINFO << "last frame y: " << cur_frame_utm_y << "  cur frame y: " <<
//     cur_frame_utm_y; reference_line_->GetFrenet(last_frame_utm_x,
//     last_frame_utm_y, last_frame_adc_heading, &last_frame_frenet_s,
//     &last_frame_frenet_l); reference_line_->GetFrenet(cur_frame_utm_x,
//     cur_frame_utm_y, cur_frame_adc_heading, &cur_frame_frenet_s,
//     &cur_frame_frenet_l); AINFO << "last l: " << last_frame_frenet_l << " cur
//     l: " << cur_frame_frenet_l; double l_diff = std::abs(cur_frame_frenet_l -
//     last_frame_frenet_l);

//     AINFO << "l diff: " << l_diff;

//     is_localization_valid_ = l_diff <
//     invalid_localization_pos_diff_threshold_;
// }

}  // namespace planning
