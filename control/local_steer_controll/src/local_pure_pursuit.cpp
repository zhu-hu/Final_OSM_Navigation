#include "local_pure_pursuit.h"

namespace controller {

LocalPurePursuit::LocalPurePursuit() {
  path_flag_ = false;

  ros::NodeHandle pnh("~");

  pnh.param("min_ref_speed", min_ref_speed_, 2.0);
  pnh.param("K_ref", K_ref_, 0.5);
  pnh.param("base_reference_dis", base_reference_dis_, 5.0);
  pnh.param("Kp_error", Kp_error_, 0.25);
  pnh.param("Ki_error", Ki_error_, 0.05);

  pnh.param("K_alpha", K_alpha_, 1.0);
  pnh.param("max_velocity", max_velocity_, 3.0);
  pnh.param("min_velocity", min_velocity_, 1.0);

  // speed = K_speed * wheel(实际输出的方向盘值，单位0.1degree) ＋　

  pnh.param("filter_param", filter_param_, 0.6);
  // 9167.3 = 10 * 180 * 16(传动比) / M_PI
  pnh.param("Kp_wheel", Kp_wheel_, 9167.3);
  pnh.param("wheel_max", wheel_max_, 4300);
  pnh.param("wheel_zero", wheel_zero_, 0);
  pnh.param("wheel_base", wheelbase_, 1.60);

  K_speed_ = (min_velocity_ - max_velocity_) / wheel_max_;

  pub_steer_cmd_ = nh.advertise<std_msgs::Float64>("/steer_cmd", 5);
  pub_speed_cmd_ = nh.advertise<std_msgs::Int32MultiArray>("/speed_cmd", 5);

  pub_target_path_ = nh.advertise<geometry_msgs::PoseArray>("/target_path", 5);
  pub_control_target_point_ = nh.advertise<visualization_msgs::MarkerArray>(
      "/visualization/control_target_point", 5);

  sub_local_trajectory_ = nh.subscribe(
      "/local_trajectory", 5, &LocalPurePursuit::LocalTrajectoryCallback, this);

  sub_speed_feedback_ = nh.subscribe("/e100/speed_feedback", 1,
                                     &LocalPurePursuit::SpeedFeedback, this);

  sub_steer_feedback_ = nh.subscribe("/e100/steer_feedback", 1,
                                     &LocalPurePursuit::SteerFeedback, this);

  sub_angular_vel_ = nh.subscribe("/imu/angular_velocity", 1,
                                  &LocalPurePursuit::ImuFeedback, this);
  timer_ = nh.createTimer(ros::Duration(0.02), &LocalPurePursuit::TimerCallback,
                          this);
  ROS_INFO("Pure Pursuit Node Init Done!!!");

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
}

LocalPurePursuit::~LocalPurePursuit() {
  ROS_INFO("PurePursuit Node ShutDown!!!");
}

//每更新一次轨迹，就做一次转向控制，转向控制的频率和轨迹规划的频率相同
void LocalPurePursuit::LocalTrajectoryCallback(
    const cyber_msgs::LocalTrajList::ConstPtr& path_in) {
  if (path_in->points.size() == 0) return;
  //规划没有出轨迹，单独对速度进行处理
  if (path_in->points.size() == 1 && path_in->points.back().mode == 32) {
    std::cout << "No trajectory, Brake!!!" << std::endl;
    std::vector<int> speed;
    speed.emplace_back(0);
    speed.emplace_back(15);  //刹车值范围是0-30，值越大刹得越急
    std_msgs::Int32MultiArray speed_msg;
    speed_msg.data = speed;
    pub_speed_cmd_.publish(speed_msg);
    return;
  }

  target_path_.header.stamp = ros::Time::now();
  target_path_.header.frame_id = "base_link";
  target_path_.poses.clear();

  target_trajectory_.clear();

  geometry_msgs::Pose traj_pose;

  for (const auto& pose : path_in->points) {
    traj_pose.position = pose.position;
    traj_pose.orientation = pose.orientation;
    TrajPoint tmp_point(traj_pose, pose.mode);
    target_path_.poses.emplace_back(traj_pose);
    target_trajectory_.emplace_back(tmp_point);
  }
  path_flag_ = true;
  current_point_index_ = 0;

  pub_target_path_.publish(target_path_);

  ComputeSteerCmd();
}

//车辆底层速度反馈
void LocalPurePursuit::SpeedFeedback(
    const cyber_msgs::VehicleSpeedFeedbackConstPtr& speed_in) {
  current_vel_ = speed_in->speed_cmps / 100.0;
}

//车辆底层转向反馈
void LocalPurePursuit::SteerFeedback(
    const cyber_msgs::VehicleSteerFeedbackConstPtr& steer_in) {
  current_steer_angle_ = steer_in->steer_0p1d / Kp_wheel_;
}

// IMU反馈
void LocalPurePursuit::ImuFeedback(
    const geometry_msgs::Vector3StampedConstPtr& imu_in) {
  current_vel_angle_ = imu_in->vector.z;
}

void LocalPurePursuit::TimerCallback(const ros::TimerEvent&) {}

void LocalPurePursuit::ComputeSteerCmd() {
  double R_current;

  if (current_vel_ < min_ref_speed_) {
    R_current = 0.0;
    current_steer_angle_ = 0.0;
    temp_steer_error_ = 0.0;
  } else if (fabs(current_vel_angle_) <
             std::numeric_limits<double>::epsilon()) {
    R_current = 0.0;
    current_steer_angle_ = 0.0;
    temp_steer_error_ = former_steer_cmd_ - current_steer_angle_;
  } else {
    R_current = current_vel_ /
                current_vel_angle_;  //向左为正，通过线速度与角速度求得转弯半径
    current_steer_angle_ = atan(wheelbase_ / R_current);
    temp_steer_error_ = former_steer_cmd_ - current_steer_angle_;
  }

  //计算控制的预描距离
  double reference_distance = 0.0;

  if (current_vel_ < min_ref_speed_)
    reference_distance = 5.0;
  else
    reference_distance = 5.0 + K_ref_ * (current_vel_ - min_ref_speed_);

  //计算控制目标对应点
  target_pose_.header.frame_id = "base_link";
  target_pose_.header.stamp = ros::Time::now();

  int target_point = current_point_index_;

  double temp_dis = std::numeric_limits<double>::max();

  for (int i = target_point; i < target_trajectory_.size(); i++) {
    double next_dis =
        abs(hypot(target_trajectory_[current_point_index_].point.position.x -
                      target_trajectory_[i].point.position.x,
                  target_trajectory_[current_point_index_].point.position.y -
                      target_trajectory_[i].point.position.y) -
            reference_distance);
    if (next_dis <= temp_dis) {
      target_point = i;
      temp_dis = next_dis;
    }
  }

  int further_target_point = target_point;
  //防止由于道路拼接导致的目标点搜索出现卡住（由于折返）
  for (int i = further_target_point + 1; i <= further_target_point + 20; i++) {
    if (i >= target_trajectory_.size()) break;

    double next_dis =
        abs(hypot(target_trajectory_[current_point_index_].point.position.x -
                      target_trajectory_[i].point.position.x,
                  target_trajectory_[current_point_index_].point.position.y -
                      target_trajectory_[i].point.position.y) -
            reference_distance);
    if (next_dis < temp_dis) {
      further_target_point = i;
      temp_dis = next_dis;
    }

    if (further_target_point != target_point) {
      target_point = further_target_point;
      ROS_INFO("Use Further Target Point!!!");
    }
  }

  //获取纯跟踪算法的预描点
  target_pose_.pose = target_trajectory_[target_point].point;

  ShowTargetPoint(target_pose_);

  //利用纯跟踪算法计算前轮转角
  //局部坐标系下自车位姿为（0,0,0）
  double delta_x = target_pose_.pose.position.x - 0.0;
  double delta_y = target_pose_.pose.position.y - 0.0;

  double L = hypot(delta_x, delta_y);

  double alpha = atan2(delta_y, delta_x) - 0.0;  //向左为正数

  if (alpha < -M_PI) alpha += 2 * M_PI;
  if (alpha > M_PI) alpha -= 2 * M_PI;

  double R_cmd;

  if (current_point_index_ == target_trajectory_.size() - 1) {  //如果没轨迹了
    R_cmd = 0.0;
    temp_steer_cmd_ = 0.0;
    former_steer_cmd_ = 0.0;
  } else if (L < std::numeric_limits<double>::epsilon() ||
             alpha < std::numeric_limits<double>::epsilon()) {
    R_cmd = 0.0;
    temp_steer_cmd_ = 0.0;
  } else {
    R_cmd = L / (2 * sin(alpha));
    temp_steer_cmd_ = atan(wheelbase_ / R_cmd);
  }

  //！！！！最后只使用纯P控制，根据航向误差算出方向盘转角的控制量
  temp_steer_cmd_ = K_alpha_ * alpha;

  //前轮转角滤波与方向盘转角计算
  final_steer_cmd_ =
      temp_steer_cmd_ +
      Kp_error_ * temp_steer_error_;  //将纯跟踪算法的计算值叠加上转向误差PI控制

  final_steer_cmd_ = filter_param_ * final_steer_cmd_ +
                     (1.0 - filter_param_) *
                         former_steer_cmd_;  //对所求得的转向控制角进行低通滤波

  //输出限幅
  if (final_steer_cmd_ > wheel_max_ / Kp_wheel_)
    final_steer_cmd_ = wheel_max_ / Kp_wheel_;
  if (final_steer_cmd_ < -wheel_max_ / Kp_wheel_)
    final_steer_cmd_ = -wheel_max_ / Kp_wheel_;

  wheel_output_.data = -1.0 * (Kp_wheel_ * final_steer_cmd_ + wheel_zero_);

  former_steer_cmd_ = final_steer_cmd_;

  std::cout << "steer_cmd : " << wheel_output_ << "deg." << std::endl;
  pub_steer_cmd_.publish(wheel_output_);

  //速度输出方式待定
  std::vector<int> speed;
  int vel = (K_speed_ * fabs(wheel_output_.data) + max_velocity_) * 100;
  std::cout << "speed : " << vel << " cm/s" << std::endl;
  speed.emplace_back(vel);
  speed.emplace_back(0);
  std_msgs::Int32MultiArray speed_msg;
  speed_msg.data = speed;
  pub_speed_cmd_.publish(speed_msg);
}

void LocalPurePursuit::ShowTargetPoint(
    const geometry_msgs::PoseStamped in_target_point) {
  visualization_msgs::MarkerArray points;
  int point_id = 0;
  visualization_msgs::Marker point;
  point.header.stamp = ros::Time::now();
  point.header.frame_id = "base_link";
  point.scale.x = 0.3;
  point.scale.y = 0.3;
  point.scale.z = 2.0;
  point.action = visualization_msgs::Marker::ADD;
  point.type = visualization_msgs::Marker::CYLINDER;
  point.lifetime = ros::Duration(0.2);

  point.pose.position.x = in_target_point.pose.position.x;
  point.pose.position.y = in_target_point.pose.position.y;
  point.pose.position.z = 1;
  point.color.b = 0;
  point.color.g = 1;
  point.color.r = 0;
  point.color.a = 1;
  points.markers.emplace_back(point);
  pub_control_target_point_.publish(points);
}
}  // namespace controller

int main(int argc, char** argv) {
  ros::init(argc, argv, "LocalPurePursuit");

  controller::LocalPurePursuit local_pure_pursuit_obj;

  return 0;
}