#include "pplqr_back_controller.h"

namespace controller {
PPLqrBackController::PPLqrBackController(ros::NodeHandle *nh) : nh_(nh) {
  sub_local_traj_ =
      nh_->subscribe("/reverse_trailer_path", 5,
                     &PPLqrBackController::LocalTrajCallback, this);

  sub_localization_ =
      new message_filters::Subscriber<cyber_msgs::LocalizationEstimate>(
          *nh_, "/localization/estimation", 1);
  sub_steer_ =
      new message_filters::Subscriber<cyber_msgs::VehicleSteerFeedback>(
          *nh_, "/e100/steer_feedback", 1);
  sub_link_angle_ = new message_filters::Subscriber<cyber_msgs::LinkAngle>(
      *nh_, "/fusion_angle_degree", 1);

  feedback_sync_ = new message_filters::Synchronizer<FeedbackPolicy>(
      FeedbackPolicy(20), *sub_localization_, *sub_steer_, *sub_link_angle_);
  feedback_sync_->registerCallback(
      boost::bind(&PPLqrBackController::FeedbackCallback, this, _1, _2, _3));

  //单位：0.1度,方向盘向左打为负，向右打为正,与实际的delta角的正负是相反的
  pub_steer_cmd_ = nh_->advertise<std_msgs::Float64>("/steer_cmd", 5);

  //单位：cm/s,速度指令前进为正，倒退为负
  pub_speed_cmd_ = nh_->advertise<std_msgs::Int32MultiArray>("/speed_cmd", 5);

  pub_rviz_nearest_point_ =
      nh_->advertise<visualization_msgs::MarkerArray>("/rviz_nearest_point", 5);

  pub_rviz_lookahead_point_ = nh_->advertise<visualization_msgs::MarkerArray>(
      "/rviz_lookahead_point", 5);

  pub_one_trajectory_finished_ =
      nh_->advertise<std_msgs::Int8>("/one_trajectory_finished", 5);

  sub_trajectories_done_ =
      nh_->subscribe("/all_trajectories_done", 5,
                     &PPLqrBackController::AllTrajectoriesDoneCallback, this);

  nh->param("SizeParam_L1", size_param_L1_, 1.62);
  nh->param("SizeParam_L2", size_param_L2_, 2.04);
  nh->param("SizeParam_M1", size_param_M1_, 0.57);
  nh->param("angle_weight", angle_weight_, 1.0);
  nh->param("steer_weight", steer_weight_, 1.0);
  nh->param("speed_weight", speed_weight_, 1.0);
  nh->param("look_ahead_dis", look_ahead_dis_, 5.0);
  nh->param("Kp", Kp_, 0.2);
  nh->param("tolerance", tolerance_, 0.01);
  nh->param("max_iteration", max_iteration_, 200);
  nh->param("filter_param", filter_param_, 0.7);
  nh->param("max_wheel_degree", max_wheel_degree_, 4500.0);
  nh->param("desired_v", desired_v_, -1.0);

  nh->param("lqr_test", lqr_test_, false);
  nh->param("fixed_desired_beta", fixed_desired_beta_, 0.1);
  nh->param("max_fabs_beta", max_fabs_beta_, 0.8);

  nh->param("use_pid", use_pid_, false);
  if (use_pid_) {
    nh->param("PID_Kp", pid_.Kp, 0.5);
    nh->param("PID_Ki", pid_.Ki, 0.05);
    nh->param("PID_anti_i_coeff", pid_.anti_i_saturation_coeff, 0.5);
    nh->param("PID_max_error_sum", pid_.max_error_sum, 0.05);
    pid_.Kd = 0.0;
  }

  if (lqr_test_) {
    //角度跟踪环节用到定时器中断
    timer_ = nh_->createTimer(ros::Duration(0.05),
                              &PPLqrBackController::TimerCallback, this);
    pub_desired_beta_ =
        nh_->advertise<std_msgs::Float64>("/desired_beta_degree", 10);
  }

  Q_(0, 0) = angle_weight_;
  R_(0, 0) = steer_weight_;
  R_(1, 1) = speed_weight_;
  R_(0, 1) = 0.0;
  R_(1, 0) = 0.0;

  max_steer_rad_ = Deg2Rad(max_wheel_degree_ / 10.0 / 16.0);

  last_delta_output_ = 0.0;

  K_ = Eigen::MatrixXd::Zero(2, 1);

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();
}

PPLqrBackController::~PPLqrBackController() {}

void PPLqrBackController::FeedbackCallback(
    const cyber_msgs::LocalizationEstimateConstPtr &localization_in,
    const cyber_msgs::VehicleSteerFeedbackConstPtr &steer_in,
    const cyber_msgs::LinkAngleConstPtr &link_angle_degree_in) {
  if (!feedback_flag_) feedback_flag_ = true;
  trailer_state_.x = localization_in->pose.position.x;
  trailer_state_.y = localization_in->pose.position.y;
  trailer_state_.theta = tf::getYaw(localization_in->pose.orientation);
  trailer_state_.velocity = localization_in->velocity.linear.x;

  trailer_state_.steer = -steer_in->steer_0p1d * M_PI / (10.0 * 180.0 * 16.0);

  trailer_state_.beta = link_angle_degree_in->link_angle_degree * M_PI / 180.0;

  //获取第二节trailer的状态
  GetSecondTrailerState();

  if (lqr_test_ == false) {
    //计算转角和速度的输出值
    ComputeSteerCmd();
  }
}

void PPLqrBackController::LocalTrajCallback(
    const cyber_msgs::LocalTrajList::ConstPtr &msg) {
  AINFO << "received trajectory!!!  "
        << "points num : " << msg->points.size();
  if (msg->points.size() == 0) return;
  if (!traj_flag_) traj_flag_ = true;
  desired_v_ = msg->points[0].velocity;
  trajectory_points_.clear();
  geometry_msgs::Pose traj_pose;
  for (const auto &point : msg->points) {
    traj_pose.position = point.position;
    traj_pose.orientation = point.orientation;
    TrajPoint tmp_point(traj_pose, point.mode);
    trajectory_points_.emplace_back(tmp_point);
  }
}

void PPLqrBackController::AllTrajectoriesDoneCallback(
    const std_msgs::BoolConstPtr &msg) {
  all_trajectories_done_ = msg->data;
}

void PPLqrBackController::TimerCallback(const ros::TimerEvent &evt) {
  if (feedback_flag_ == false) {
    AINFO << "feedback not init!!!";
    return;
  }

  if (use_pid_) {
    ComputeSteerCmdUsingPID();
    return;
  }

  desired_beta_ = fixed_desired_beta_;

  if (desired_beta_ > max_fabs_beta_) desired_beta_ = max_fabs_beta_;

  if (desired_beta_ < -max_fabs_beta_) desired_beta_ = -max_fabs_beta_;

  std_msgs::Float64 desire_beta_degree;
  desire_beta_degree.data = Rad2Deg(desired_beta_);
  pub_desired_beta_.publish(desire_beta_degree);

  double R_0 = (size_param_L2_ + size_param_M1_ / cos(desired_beta_)) /
                   sin(fabs(desired_beta_) + EPSILON) -
               size_param_M1_ * tan(fabs(desired_beta_));
  desired_delta_ = Sign(desired_beta_) * atan2(size_param_L1_, R_0);

  AINFO << "desired beta : " << desired_beta_;
  AINFO << "desired delta : " << desired_delta_;

  double desire_v = desired_v_;

  double delta_t = ros::Time::now().toSec() - last_time_;

  last_time_ = ros::Time::now().toSec();

  A_ << -delta_t * desire_v *
                (size_param_M1_ * tan(desired_delta_) * sin(desired_beta_) +
                 size_param_L1_ * cos(desired_beta_)) /
                (size_param_L1_ * size_param_L2_) +
            1.0;

  B_ << delta_t * desire_v *
            ((1.0 / size_param_L1_ + size_param_M1_ * cos(desired_beta_) /
                                         size_param_L1_ / size_param_L2_) /
             (cos(desired_delta_) * cos(desired_delta_))),
      delta_t *
          (tan(desired_delta_) / size_param_L1_ *
               (1.0 + size_param_M1_ * cos(desired_beta_) / size_param_L2_) -
           sin(desired_beta_) / size_param_L2_);
  X_ << trailer_state_.beta - desired_beta_;

  common::math::SolveLQRProblem(A_, B_, Q_, R_, tolerance_, max_iteration_,
                                &K_);
  AINFO << "delta_t : " << delta_t;
  AINFO << "matrix A : " << A_;
  AINFO << "matrix B : " << B_;
  AINFO << "matrix Q : " << Q_;
  AINFO << "matrix R : " << R_;

  AINFO << "matrix X : " << X_;

  AINFO << "matrix_k : " << K_;
  if (std::isnan(K_(0, 0)) || std::isnan(K_(1, 0))) return;
  double delta_delta_output = -(K_ * X_)(0, 0);
  AINFO << "delta_delta_output : " << delta_delta_output;

  double delta_v_output = -(K_ * X_)(1, 0);

  double delta_delta_output_now = desired_delta_ + delta_delta_output;

  AINFO << "delta_delta_output_now : " << delta_delta_output_now;

  //前轮转角最终输出值
  double final_delta_output = filter_param_ * delta_delta_output_now +
                              (1.0 - filter_param_) * last_delta_output_;
  AINFO << "filter_param : " << filter_param_;
  AINFO << "last_delta_output : " << last_delta_output_;
  AINFO << "final_delta_output : " << final_delta_output;
  //限幅
  final_delta_output =
      LimitAmplitude(-max_steer_rad_, max_steer_rad_, final_delta_output);

  AINFO << "final_delta_output : " << final_delta_output;

  //发布方向盘转角的值
  std_msgs::Float64 steer_cmd;
  steer_cmd.data = -Rad2Deg(final_delta_output) * 16.0 * 10.0;
  pub_steer_cmd_.publish(steer_cmd);

  AINFO << "steer angle : " << final_delta_output << " rad";
  AINFO << "wheel angle : " << steer_cmd.data << " degree";

  //速度输出值
  double final_speed_output = desire_v + delta_v_output;
  //暂不用lqr的速度结果
  std::vector<int> speed;
  speed.emplace_back(desire_v * 100);
  speed.emplace_back(0);
  std_msgs::Int32MultiArray speed_msg;
  speed_msg.data = speed;
  pub_speed_cmd_.publish(speed_msg);

  AINFO << "final speed output : " << final_speed_output;

  last_time_ = ros::Time::now().toSec();

  last_delta_output_ = final_delta_output;
}

//获取拖挂trailer的状态(x,y,theta,v)
void PPLqrBackController::GetSecondTrailerState() {
  second_trailer_state_.theta =
      NormalAngle(trailer_state_.theta - trailer_state_.beta);
  second_trailer_state_.x =
      trailer_state_.x - (size_param_M1_ * cos(trailer_state_.theta) +
                          size_param_L2_ * cos(second_trailer_state_.theta));
  second_trailer_state_.y =
      trailer_state_.y - (size_param_M1_ * sin(trailer_state_.theta) +
                          size_param_L2_ * sin(second_trailer_state_.theta));
  second_trailer_state_.velocity =
      trailer_state_.velocity *
      (size_param_M1_ * sin(trailer_state_.beta) * tan(trailer_state_.steer) /
           size_param_L1_ +
       cos(trailer_state_.beta));
  second_trailer_state_.steer = 0.0;
  second_trailer_state_.beta = 0.0;
}

//找到参考轨迹上离当前第二节trailer后轴中心位置距离最近的点
int PPLqrBackController::GetNearestTrajPoint() {
  if (!traj_flag_) return -1;
  double min_dis =
      sqrt(pow(second_trailer_state_.x - trajectory_points_[0].point.position.x,
               2.0) +
           pow(second_trailer_state_.y - trajectory_points_[0].point.position.y,
               2.0));
  int nearest_index = 0;

  for (int i = 0; i < trajectory_points_.size(); i++) {
    double distance = sqrt(
        pow(second_trailer_state_.x - trajectory_points_[i].point.position.x,
            2.0) +
        pow(second_trailer_state_.y - trajectory_points_[i].point.position.y,
            2.0));
    if (distance < min_dis) {
      nearest_index = i;
      min_dis = distance;
    }
  }

  geometry_msgs::Point nearest_point;
  nearest_point = trajectory_points_[nearest_index].point.position;

  ShowPointInRviz(pub_rviz_nearest_point_, nearest_point, "blue");

  return nearest_index;
}

void PPLqrBackController::PurePursuit() {
  if (traj_flag_ == false || feedback_flag_ == false) return;

  //找到跟踪的预描点
  int nearest_index = GetNearestTrajPoint();
  target_point_index_ = nearest_index;
  double real_look_ahead_dis = 0.0;
  for (int i = nearest_index; i < trajectory_points_.size(); i++) {
    double distance = sqrt(
        pow(second_trailer_state_.x - trajectory_points_[i].point.position.x,
            2.0) +
        pow(second_trailer_state_.y - trajectory_points_[i].point.position.y,
            2.0));
    if (distance >= look_ahead_dis_) {
      real_look_ahead_dis = distance;
      target_point_index_ = i;
      break;
    }

    if (i == trajectory_points_.size() - 1) {
      real_look_ahead_dis = distance;
      target_point_index_ = i;
      break;
    }
  }

  geometry_msgs::Point lookahead_point =
      trajectory_points_[target_point_index_].point.position;

  ShowPointInRviz(pub_rviz_lookahead_point_, lookahead_point, "red");

  double theta_e = NormalAngle(
      second_trailer_state_.theta -
      atan2(second_trailer_state_.y -
                trajectory_points_[target_point_index_].point.position.y,
            second_trailer_state_.x -
                trajectory_points_[target_point_index_].point.position.x));
  double R_1 = real_look_ahead_dis / (2 * fabs(sin(theta_e + EPSILON)));
  double R_0 =
      sqrt(pow(size_param_L2_, 2.0) + pow(R_1, 2.0) - pow(size_param_M1_, 2.0));

  //符号需要再确定一下：符号由theta_e的符号来确定,平衡点的角度的符号均和theta_e的符号保持一致
  double desired_beta_pp = Sign(theta_e) * (atan2(size_param_M1_ / R_0, 1.0) +
                                            atan2(size_param_L2_ / R_1, 1.0));

  desired_beta_ =
      desired_beta_pp + Kp_ * (desired_beta_pp - trailer_state_.beta);

  if (desired_beta_ > max_fabs_beta_) desired_beta_ = max_fabs_beta_;

  if (desired_beta_ < -max_fabs_beta_) desired_beta_ = -max_fabs_beta_;

  desired_delta_ = Sign(theta_e) * atan2(size_param_L1_ / R_0, 1.0);

  AERROR << "desired_beta : " << desired_beta_;
  AERROR << "desired_delta : " << desired_delta_;
}

void PPLqrBackController::ComputeSteerCmd() {
  if (traj_flag_ == false) {
    AINFO << "trajectory not init!!!";
    return;
  }

  if (feedback_flag_ == false) {
    AINFO << "feedback not init!!!";
    return;
  }

  //如果拖挂车后轴中心与轨迹终点很接近，则停止速度控制，且不必进行后续计算，直接退出此函数
  int nearest_index = GetNearestTrajPoint();
  if (nearest_index == trajectory_points_.size() - 1) {
    //踩刹车停车
    std::vector<int> speed;
    speed.emplace_back(0);
    speed.emplace_back(10);
    std_msgs::Int32MultiArray speed_msg;
    speed_msg.data = speed;
    pub_speed_cmd_.publish(speed_msg);

    std_msgs::Int8 msg;
    //不是最后一条轨迹的时候，需要发布完成当前段控制的消息
    if (all_trajectories_done_ == false) {
      msg.data = 1;
    } else {
      msg.data = 2;
    }
    pub_one_trajectory_finished_.publish(msg);
    //某一段轨迹控制完成之后，reset　traj_flag!
    traj_flag_ = false;
    return;
  }

  //根据预描的方式获取轨迹上的参考控制点，以及平衡态的两个角度
  PurePursuit();

  double desire_v = desired_v_;

  double delta_t = ros::Time::now().toSec() - last_time_;

  A_ << -delta_t * desire_v *
                (size_param_M1_ * tan(desired_delta_) * sin(desired_beta_) +
                 size_param_L1_ * cos(desired_beta_)) /
                (size_param_L1_ * size_param_L2_) +
            1.0;

  B_ << delta_t * desire_v *
            ((1.0 / size_param_L1_ + size_param_M1_ * cos(desired_beta_) /
                                         size_param_L1_ / size_param_L2_) /
             (cos(desired_delta_) * cos(desired_delta_))),
      delta_t *
          (tan(desired_delta_) / size_param_L1_ *
               (1.0 + size_param_M1_ * cos(desired_beta_) / size_param_L2_) -
           sin(desired_beta_) / size_param_L2_);
  X_ << trailer_state_.beta - desired_beta_;

  common::math::SolveLQRProblem(A_, B_, Q_, R_, tolerance_, max_iteration_,
                                &K_);

  double delta_delta_output = -(K_ * X_)(0, 0);

  double delta_v_output = -(K_ * X_)(1, 0);

  double delta_delta_output_now = desired_delta_ + delta_delta_output;

  //前轮转角最终输出值
  double final_delta_output = filter_param_ * delta_delta_output_now +
                              (1.0 - filter_param_) * last_delta_output_;
  //限幅
  final_delta_output =
      LimitAmplitude(-max_steer_rad_, max_steer_rad_, final_delta_output);

  //发布方向盘转角的值
  std_msgs::Float64 steer_cmd;
  steer_cmd.data = -Rad2Deg(final_delta_output) * 16.0 * 10.0;
  pub_steer_cmd_.publish(steer_cmd);

  AINFO << "steer angle : " << final_delta_output << " rad";
  AINFO << "wheel angle : " << steer_cmd.data << " degree";

  //速度输出值
  double final_speed_output = desire_v + delta_v_output;

  //发布速度输出值，暂时不用lqr的结果，直接将期望速度发布到车辆底层
  //之后需要根据是否到达控制终点附近对速度输出进行调整
  std::vector<int> speed;
  speed.emplace_back(desire_v * 100);
  speed.emplace_back(0);
  std_msgs::Int32MultiArray speed_msg;
  speed_msg.data = speed;
  pub_speed_cmd_.publish(speed_msg);

  last_time_ = ros::Time::now().toSec();

  last_delta_output_ = final_delta_output;
}

void PPLqrBackController::ComputeSteerCmdUsingPID() {
  desired_beta_ = fixed_desired_beta_;

  double current_beta = trailer_state_.beta;

  double beta_error = desired_beta_ - current_beta;

  //抗积分饱和
  if (fabs(beta_error) < fabs(desired_beta_) * pid_.anti_i_saturation_coeff)
    pid_.error_sum += beta_error;

  //积分限幅
  if (pid_.error_sum > pid_.max_error_sum) pid_.error_sum = pid_.max_error_sum;
  if (pid_.error_sum < -pid_.max_error_sum)
    pid_.error_sum = -pid_.max_error_sum;

  double delta_rad = pid_.Kp * beta_error + pid_.Ki * pid_.Ki * pid_.error_sum;
  AINFO << "Kp : " << pid_.Kp << "  Ki : " << pid_.Ki << "  Kd" << pid_.Kd;
  AINFO << "max error sum : " << pid_.max_error_sum
        << "  anti_i_coeff : " << pid_.anti_i_saturation_coeff;
  AINFO << "beta_error : " << beta_error << "  error_sum : " << pid_.error_sum;

  AINFO << "delta_rad : " << delta_rad;
  delta_rad = LimitAmplitude(-max_steer_rad_, max_steer_rad_, delta_rad);
  AINFO << "delta_rad : " << delta_rad;

  double wheel_degree = -Rad2Deg(delta_rad) * 16.0 * 10.0;

  AINFO << "wheel output : " << wheel_degree << " Deg";

  std_msgs::Float64 wheel_msg;
  wheel_msg.data = wheel_degree;
  pub_steer_cmd_.publish(wheel_msg);
}

void PPLqrBackController::ShowPointInRviz(const ros::Publisher &pub,
                                          const geometry_msgs::Point &point,
                                          const std::string &color) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker point_marker;
  int id = 0;
  point_marker.header.frame_id = "world";
  point_marker.header.stamp = ros::Time::now();
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.type = visualization_msgs::Marker::CYLINDER;
  point_marker.color.r = 0.0f;
  point_marker.color.g = 1.0f;
  point_marker.color.b = 0.0f;
  if (color == "red") {
    point_marker.color.r = 1.0f;
    point_marker.color.g = 0.0f;
    point_marker.color.b = 0.0f;
  }
  if (color == "blue") {
    point_marker.color.r = 0.0f;
    point_marker.color.g = 0.0f;
    point_marker.color.b = 1.0f;
  }

  if (color == "yellow") {
    point_marker.color.r = 1.0f;
    point_marker.color.g = 1.0f;
    point_marker.color.b = 0.0f;
  }
  point_marker.color.a = 1.0f;
  point_marker.scale.x = 0.5;
  point_marker.scale.y = 0.5;
  point_marker.scale.z = 1.0;
  point_marker.lifetime = ros::Duration(0.0);
  point_marker.id = id++;
  point_marker.pose.position = point;

  marker_array.markers.emplace_back(point_marker);

  pub.publish(marker_array);
}

}  // namespace controller