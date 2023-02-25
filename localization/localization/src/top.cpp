/**
 *top.h
 *brief:top layer of localization pipeline
 *author:Chen Xiaofeng
 *date:20191028
 **/

#include "top.h"

Top::Top(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
{
  //   nh_priv.param<bool>("command_no_gps",command_no_gps_,false);
  nh_priv_ = nh_priv;
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
  nh_priv.param<double>("cali_x_", cali_x_, -0.46);
  nh_priv.param<double>("cali_y_", cali_y_, 0.32);
  nh_priv.param<double>("car_heading_error", car_heading_error, 0.01);
  nh_priv.param<int>("err_cnt", err_cnt, 90);

  nh_priv.param<double>("GLOBAL_ZERO_X_", GLOBAL_ZERO_X_, 355000);
  nh_priv.param<double>("GLOBAL_ZERO_Y_", GLOBAL_ZERO_Y_, 2700000);

  nh_priv.param<string>("localization_region", localization_region_, "SJTU");
  if (localization_region_ == "qingdao")
    localization_region_num_ = 0;
  else if (localization_region_ == "liuzhou")
    localization_region_num_ = 1;
  else if (localization_region_ == "SJTU")
    localization_region_num_ = 2;
  else if (localization_region_ == "changshu")
    localization_region_num_ = 3;
  else
  {
    ROS_ERROR("localization_region invalid!");
    return;
  }

  fusion_.getErrParam(gps_err_fix_, imu_err_);
  // heading_fusion_.getErrParam(imu_err_);

  pub_localization_estimation_ = nh.advertise<cyber_msgs::LocalizationEstimate>(
      "/localization/estimation", 1);
  if (localization_debug_)
  {
    pub_localization_estimation_angle_ =
        nh.advertise<std_msgs::Float64>("/localization/estimation_angle", 1);
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

    pub_gps_angular_ =
        nh.advertise<std_msgs::Float32>("/localization_debug/gps_angular", 1);
    pub_slam_angular_ = nh.advertise<std_msgs::Float64>(
        "/localization_debug/slam_angular", 1);
    pub_pose_diff_ = nh.advertise<geometry_msgs::Pose2D>("/diff_pose", 2);
  }

  vel_sub_ =
      nh.subscribe("/e100/speed_feedback", 1000, &Top::vel_callback, this);
  if (use_steer_)
    steer_sub_ =
        nh.subscribe("/e100/steer_feedback", 1000, &Top::steer_callback, this);
  else
    sub_imu_ =
        nh.subscribe("/imu/angular_velocity", 1000, &Top::imu_callback, this);
  //   command_sub_ = nh.subscribe("/localize_mode", 1000,
  //   &Top::command_callback, this);

  sub_gps_fix_ = new message_filters::Subscriber<sensor_msgs::NavSatFix>(
      nh, "/strong/fix", 1);
  sub_gps_heading_ = new message_filters::Subscriber<cyber_msgs::Heading>(
      nh, "/strong/heading", 1);
  sub_gps_status_ = new message_filters::Subscriber<cyber_msgs::GPGGA_MSG>(
      nh, "/strong/raw_data", 1);
  gps_sync_ = new message_filters::Synchronizer<GpsPolicy>(
      GpsPolicy(10), *sub_gps_fix_, *sub_gps_heading_, *sub_gps_status_);
  gps_sync_->registerCallback(
      boost::bind(&Top::gps_callback, this, _1, _2, _3));

  sub_slam_fix_ = new message_filters::Subscriber<sensor_msgs::NavSatFix>(
      nh, "/slam/gps/fix", 1);
  sub_slam_heading_ = new message_filters::Subscriber<sensor_msgs::Imu>(
      nh, "/slam/gps/heading", 1);
  slam_sync_ = new message_filters::Synchronizer<SlamPolicy>(
      SlamPolicy(10), *sub_slam_fix_, *sub_slam_heading_);
  slam_sync_->registerCallback(boost::bind(&Top::slam_callback, this, _1, _2));

  filter_timer_ =
      nh.createTimer(ros::Duration(0.05), &Top::filter_callback, this);

  ROS_INFO("Localization Pipeline begin!");
}

Top::~Top()
{
  delete sub_gps_fix_;
  delete sub_gps_heading_;
  delete sub_gps_status_;
  delete gps_sync_;
  delete sub_slam_fix_;
  delete sub_slam_heading_;
  delete slam_sync_;
}

double Top::get_time_now() { return ros::Time::now().toSec() - t_diff_; }

// void Top::command_callback(const std_msgs::Int8ConstPtr &bool_in)
// {
//     if(bool_in->data > 0) command_no_gps_ = true;
//     else command_no_gps_ = false;
// }

void Top::vel_callback(const cyber_msgs::VehicleSpeedFeedbackConstPtr &vel_in)
{
  if (!have_inited_gps_)
  {
    ROS_INFO("wait for GPS input!");
    return;
  }

  have_inited_vel_ = true;

  curr_vel_ = (double)(vel_in->speed_cmps) / 100.0;

  Eigen::Vector2d Z_vel;
  Eigen::Matrix2d R_vel;
  Z_vel << curr_vel_, curr_yaw_rate_;
  R_vel << pow(speed_err_, 2), 0, 0, pow(imu_err_, 2);

  t_vel_ = get_time_now();
  if (fabs(curr_vel_) > 0.01)
    fusion_.velStateUpdate(Z_vel, R_vel, t_vel_);
  else
    fusion_.timeUpdate(t_vel_);
}

void Top::imu_callback(const geometry_msgs::Vector3StampedConstPtr &imu_in)
{
  curr_yaw_rate_ = imu_in->vector.z;
  if (curr_yaw_rate_ < 0.00001 && curr_yaw_rate_ > -0.00001)
    curr_yaw_rate_ = 0.00001;
  angular_velocity_.x = 0;
  angular_velocity_.y = 0;
  angular_velocity_.z = curr_yaw_rate_;
  // ROS_INFO("yaw rate from imu: %f", curr_yaw_rate_);
}

void Top::steer_callback(
    const cyber_msgs::VehicleSteerFeedbackConstPtr &steer_in)
{
  double angle = -steer_in->steer_0p1d / 10.0 * M_PI / 180.0 /
                 16.0; // 16.0 是方向盘转角和前轮转角的比例
  double delta_theta = (curr_vel_ * 0.01) * tan(angle) / 1.6;
  curr_yaw_rate_ = delta_theta / 0.01;
  if (curr_yaw_rate_ < 0.00001 && curr_yaw_rate_ > -0.00001)
    curr_yaw_rate_ = 0.00001;
  angular_velocity_.x = 0;
  angular_velocity_.y = 0;
  angular_velocity_.z = curr_yaw_rate_;
  // ROS_INFO("yaw rate from steer: %f", curr_yaw_rate_);
}

void Top::gps_callback(const sensor_msgs::NavSatFixConstPtr &gps_in,
                       const cyber_msgs::HeadingConstPtr &heading_in,
                       const cyber_msgs::GPGGA_MSGConstPtr &status_in)
{
  double yaw = heading_in->data;
  if (yaw >= M_PI)
    yaw -= 2 * M_PI;
  else if (yaw <= -M_PI)
    yaw += 2 * M_PI;
  global_heading_ = yaw;

  MapFrame obs_pos(localization_region_num_);
  obs_pos.GPS2MapFrame(*gps_in);
  double obs_yaw = yaw;
//   if(last_time != -1){
//     predict_yaw_ = output_yaw_ + (get_time_now() - last_time) * curr_yaw_rate_;
//   }
  bool heading_valid = true;
  last_time = get_time_now();

  if (obs_yaw < -M_PI || obs_yaw > M_PI ||  isinf(obs_yaw))
  {
    // std::cout<<"obs_yaw="<<obs_yaw/M_PI*180<<std::endl;
    ROS_ERROR("gps heading invalid");
    return;
  }else if(isnan(obs_yaw)){
      obs_yaw = output_yaw_;
      heading_valid = false;
      ROS_WARN("gps heading nan, use predict haeding");
  }

  if (localization_debug_)
  {

    std_msgs::Float32 yaw_data;
    yaw_data.data = 180.0 * yaw / M_PI;
    pub_gps_angular_.publish(yaw_data);

    geometry_msgs::PoseStamped original_pose;
    original_pose.header.frame_id = "world";
    original_pose.header.stamp.fromSec(t_gps_);
    char zone;
    LLtoUTM(gps_in->latitude, gps_in->longitude, original_pose.pose.position.y,
            original_pose.pose.position.x, &zone, GLOBAL_ZERO_X_, GLOBAL_ZERO_Y_);
    original_pose.pose.position.z = 0;
    original_pose.pose.orientation = tf::createQuaternionMsgFromYaw(obs_yaw);

    // 把安装在左侧的主天线定位位置平移到后轴中心
    original_pose.pose.position.x +=
        cali_x_ * sin(obs_yaw) + cali_y_ * cos(obs_yaw);
    original_pose.pose.position.y +=
        -cali_x_ * cos(obs_yaw) + cali_y_ * sin(obs_yaw);

    tf::Transform transform_gps;
    tf::Quaternion q_gps;
    transform_gps.setOrigin(tf::Vector3(original_pose.pose.position.x,
                                        original_pose.pose.position.y,
                                        original_pose.pose.position.z));
    q_gps.setRPY(0, 0, obs_yaw);
    transform_gps.setRotation(q_gps);
    br_gps_.sendTransform(tf::StampedTransform(transform_gps, ros::Time::now(),
                                               "map", "original_gps"));

    geometry_msgs::Point gps_marker = original_pose.pose.position;
    gps_marker_traj_.header.frame_id = "map";
    gps_marker_traj_.header.stamp.fromSec(t_gps_);
    gps_marker_traj_.ns = "original_gps";
    gps_marker_traj_.id = 100;
    gps_marker_traj_.type = 6;
    gps_marker_traj_.action = visualization_msgs::Marker::ADD;
    gps_marker_traj_.scale.x = 0.02;
    gps_marker_traj_.scale.y = 0.02;
    gps_marker_traj_.scale.z = 0.02;
    gps_marker_traj_.points.push_back(gps_marker);
    if (gps_marker_traj_.points.size() > 500)
      gps_marker_traj_.points.erase(gps_marker_traj_.points.begin());
    std_msgs::ColorRGBA gps_color;
    gps_color.r = 1;
    gps_color.g = 0;
    gps_color.b = 0;
    gps_color.a = 1;
    gps_marker_traj_.colors.push_back(gps_color);
    if (gps_marker_traj_.colors.size() > 500)
      gps_marker_traj_.colors.erase(gps_marker_traj_.colors.begin());
    pub_gps_marker_.publish(gps_marker_traj_);

    gps_poses_.poses.push_back(original_pose.pose);
    if (gps_poses_.poses.size() > 500)
      gps_poses_.poses.erase(gps_poses_.poses.begin());
    gps_poses_.header.stamp.fromSec(t_gps_);
    gps_poses_.header.frame_id = "map";
    pub_gps_pose_.publish(gps_poses_);
  }

  double MAPFRAME_OBS_ERR = gps_err_fix_;
  double OBS_YAW_ERR = gps_yaw_err_fix_;
//   if (status_in->status == 4)
//     OBS_YAW_ERR = gps_yaw_err_fix_;
//   else
//     OBS_YAW_ERR = gps_yaw_err_normal_;

  // if (status_in->status == 4 && status_in->num_satellites > 10) {
  //   if (gps_warning_count > 0) gps_warning_count--;
  //   if (gps_error_count > 0) gps_error_count--;
  // } else if (status_in->status == 5 ||
  //            (status_in->status == 4 && status_in->num_satellites > 5)) {
  //   if (gps_warning_count < 100) gps_warning_count++;
  //   if (gps_error_count > 0) gps_error_count--;
  // } else {
  //   if (gps_warning_count < 100) gps_warning_count++;
  //   if (gps_error_count < 100) gps_error_count++;
  // }

  if (gps_error_count >= err_cnt)
  {
    // 报error，发出异常信号
    if (have_inited_gps_ && !command_no_gps_)
    {
      have_inited_gps_ = false;
      have_inited_vel_ = false;
      fusion_.Reset();
      // heading_fusion_.Reset();
    }

    if (!command_no_gps_)
    {
      cyber_msgs::LocalizationEstimate localization_estimation;
      localization_estimation.status = 0;
      pub_localization_estimation_.publish(localization_estimation);
      ROS_ERROR("GPS ERROR, Waiting for Recover");
    }

    return;
  }
//   else if (gps_warning_count > 0 || gps_error_count > 0)
//   {
//     // 报warning，还是使用GPS校正，但置信度低点
//     ROS_ERROR("GPS WARNING");
//     MAPFRAME_OBS_ERR = 0.03;
//     OBS_YAW_ERR = 0.01;
//     // return;
//   }

  if (command_no_gps_)
    return;

  //     ROS_WARN("x: %f, y: %f", obs_pos.x, obs_pos.y);
  // char zone;
  // double tx, ty;
  // LLtoUTM(gps_in->latitude, gps_in->longitude, ty, tx, &zone);
  // ROS_WARN("utm x: %f, y: %f", tx, ty);
  // 把安装在左侧的主天线定位位置平移到后轴中心
  obs_pos.x += cali_x_ * sin(obs_yaw) + cali_y_ * cos(obs_yaw);
  obs_pos.y += -cali_x_ * cos(obs_yaw) + cali_y_ * sin(obs_yaw);

  if (gps_in->latitude < 1 || gps_in->latitude > 90 ||
      isnan(gps_in->latitude) || isinf(gps_in->latitude))
  {
    ROS_INFO("gps latitude invalid");
    return;
  }
  if (gps_in->longitude < 1 || gps_in->longitude > 180 ||
      isnan(gps_in->longitude) || isinf(gps_in->longitude))
  {
    ROS_INFO("gps longitude invalid");
    return;
  }
  if (obs_yaw < -M_PI || obs_yaw > M_PI || isnan(obs_yaw) || isinf(obs_yaw))
  {
    ROS_INFO("gps heading invalid");
    return;
  }

  Eigen::Vector3d Z_gps;
  Eigen::Matrix3d R_gps;
  Z_gps << obs_pos.x, obs_pos.y, obs_yaw;
  R_gps << pow(MAPFRAME_OBS_ERR, 2), 0, 0, 0, pow(MAPFRAME_OBS_ERR, 2), 0, 0, 0,
      pow(OBS_YAW_ERR, 2);

  Eigen::Vector2d Z_gps_position;
  Eigen::Matrix2d R_gps_position;
  Z_gps_position << obs_pos.x, obs_pos.y;
  R_gps_position << pow(MAPFRAME_OBS_ERR, 2), 0, 0, pow(MAPFRAME_OBS_ERR, 2);

  t_gps_ = get_time_now();

  if (!command_no_gps_)
  {
    if (localization_fail_cnt_ > 0)
    {
      localization_fail_cnt_--;
      return;
    }

    if (!have_inited_gps_)
    {
      have_inited_gps_ = true;
      t_diff_ = ros::Time::now().toSec() - gps_in->header.stamp.toSec();
      fusion_.gpsStateUpdate(Z_gps, R_gps, t_gps_);
      // heading_fusion_.headingUpdate(obs_yaw, car_heading_error, t_gps_);
      ROS_INFO("init by gps %f,%f", obs_pos.x, obs_pos.y);
      return;
    }
    if (status_in->num_satellites > 5)
    {
        std::cout<<"abs(yaw - output_yaw_)="<<abs(yaw - output_yaw_)<<std::endl;
        if (fabs(curr_vel_) > 0.01){
            if(heading_valid && abs(yaw - output_yaw_) < 0.2)
                fusion_.gpsStateUpdate(Z_gps, R_gps, t_gps_);
            else
                fusion_.gpsStateUpdate(Z_gps_position, R_gps_position, t_gps_);
        }

      else
        fusion_.timeUpdate(t_gps_);
      // heading_fusion_.headingUpdate(obs_yaw, car_heading_error, t_gps_);
      gps_with_problem_ = false;
      ROS_INFO("update by gps in %f", MAPFRAME_OBS_ERR);
    }
    else
    {
      gps_with_problem_ = true;
      ROS_INFO("gps with problem by number of satellites.");
    }
  }
}

void Top::slam_callback(const sensor_msgs::NavSatFixConstPtr &slam_in,
                        const sensor_msgs::ImuConstPtr &heading_in)
{
  command_no_gps_ = true;
  MapFrame obs_pos(localization_region_num_);
  obs_pos.GPS2MapFrame(*slam_in);

  double roll, pitch, yaw;
  tf::Matrix3x3(
      tf::Quaternion(heading_in->orientation.x, heading_in->orientation.y,
                     heading_in->orientation.z, heading_in->orientation.w))
      .getRPY(roll, pitch, yaw);
  if (yaw >= M_PI)
    yaw -= 2 * M_PI;
  else if (yaw <= -M_PI)
    yaw += 2 * M_PI;
  double obs_yaw = yaw;

  if (slam_in->latitude < 1 || slam_in->latitude > 90 ||
      isnan(slam_in->latitude) || isinf(slam_in->latitude))
  {
    ROS_INFO("slam latitude invalid");
    return;
  }
  if (slam_in->longitude < 1 || slam_in->longitude > 180 ||
      isnan(slam_in->longitude) || isinf(slam_in->longitude))
  {
    ROS_INFO("slam longitude invalid");
    return;
  }
  if (obs_yaw < -M_PI || obs_yaw > M_PI || isnan(obs_yaw) || isinf(obs_yaw))
  {
    ROS_INFO("slam heading invalid");
    return;
  }

  double MAPFRAME_SLAM_ERR_X, MAPFRAME_SLAM_ERR_Y, MAPFRAME_SLAM_ERR,
      SLAM_YAW_ERR;
  MAPFRAME_SLAM_ERR_X = slam_in->position_covariance[0];
  MAPFRAME_SLAM_ERR_Y = slam_in->position_covariance[4];
  if (slam_in->altitude > slam_fix_param_)
  {
    MAPFRAME_SLAM_ERR = slam_err_fix_;
    SLAM_YAW_ERR = slam_yaw_err_fix_;
  }
  else
  {
    MAPFRAME_SLAM_ERR = slam_err_normal_;
    SLAM_YAW_ERR = slam_yaw_err_normal_;
  }

  Eigen::Vector3d Z_slam;
  Eigen::Matrix3d R_slam;
  Z_slam << obs_pos.x, obs_pos.y, obs_yaw;
  R_slam << pow(MAPFRAME_SLAM_ERR, 2), 0, 0, 0, pow(MAPFRAME_SLAM_ERR, 2), 0, 0,
      0, pow(SLAM_YAW_ERR, 2);

  t_slam_ = get_time_now();

  if (command_no_gps_ || gps_with_problem_)
  {
    if (!have_inited_gps_)
    {
      have_inited_gps_ = true;
      t_diff_ = ros::Time::now().toSec() - slam_in->header.stamp.toSec();
      fusion_.gpsStateUpdate(Z_slam, R_slam, t_slam_);
      // heading_fusion_.headingUpdate(obs_yaw, car_heading_error, t_slam_);
      ROS_INFO("init by slam %f,%f", obs_pos.x, obs_pos.y);
      return;
    }

    // if(slam_in->altitude > slam_error_param_)
    // {

    if (fabs(curr_vel_) > 0.01)
    {
      diff_pose_ = fusion_.slamStateUpdate(Z_slam, R_slam, t_slam_);
      //定位失效检测模块
      if (enable_manual_reset)
      {
        if ((diff_pose_.x > TRANS_THRED || diff_pose_.x < -TRANS_THRED || diff_pose_.y > TRANS_THRED || diff_pose_.y < -TRANS_THRED || diff_pose_.theta > ROTA_THRED || diff_pose_.theta < -ROTA_THRED))
          localization_fail_cnt_++;
        else if (localization_fail_cnt_ > 0 && localization_fail_cnt_ < LOCALIZATION_FAIL_NUM_ + 1)
          localization_fail_cnt_--;
        if (localization_fail_cnt_ > LOCALIZATION_FAIL_NUM_)
        {
          localization_fail_cnt_ = 200;
          cyber_msgs::LocalizationEstimate localization_estimation;
          localization_estimation.status = 0;
          pub_localization_estimation_.publish(localization_estimation);
        }
        if (localization_debug_)
          pub_pose_diff_.publish(diff_pose_);
      }
    }
    else
      fusion_.timeUpdate(t_slam_);
    // heading_fusion_.headingUpdate(obs_yaw, car_heading_error, t_slam_);
    ROS_INFO("update by slam in %f", MAPFRAME_SLAM_ERR);
    // }
    // else
    // {
    //     ROS_INFO("slam with problem");
    // }
  }

  if (localization_debug_)
  {
    geometry_msgs::PoseStamped slam_pose;
    slam_pose.header.frame_id = "world";
    slam_pose.header.stamp.fromSec(t_slam_);
    char zone;
    LLtoUTM(slam_in->latitude, slam_in->longitude, slam_pose.pose.position.y,
            slam_pose.pose.position.x, &zone, GLOBAL_ZERO_X_, GLOBAL_ZERO_Y_);
    slam_pose.pose.position.z = 0;
    slam_pose.pose.orientation = tf::createQuaternionMsgFromYaw(obs_yaw);

    std_msgs::Float64 yaw_data;
    yaw_data.data = 180.0 * obs_yaw / M_PI;
    pub_slam_angular_.publish(yaw_data);

    tf::Transform transform_slam;
    tf::Quaternion q_slam;
    transform_slam.setOrigin(tf::Vector3(slam_pose.pose.position.x,
                                         slam_pose.pose.position.y,
                                         slam_pose.pose.position.z));
    q_slam.setRPY(0, 0, obs_yaw);
    transform_slam.setRotation(q_slam);
    br_slam_.sendTransform(
        tf::StampedTransform(transform_slam, ros::Time::now(), "map", "slam"));

    geometry_msgs::Point slam_marker = slam_pose.pose.position;
    slam_marker_traj_.header.frame_id = "map";
    slam_marker_traj_.header.stamp.fromSec(t_slam_);
    slam_marker_traj_.ns = "slam";
    slam_marker_traj_.id = 101;
    slam_marker_traj_.type = 6;
    slam_marker_traj_.action = visualization_msgs::Marker::ADD;
    slam_marker_traj_.scale.x = 0.02;
    slam_marker_traj_.scale.y = 0.02;
    slam_marker_traj_.scale.z = 0.02;
    slam_marker_traj_.points.push_back(slam_marker);
    if (slam_marker_traj_.points.size() > 500)
      slam_marker_traj_.points.erase(slam_marker_traj_.points.begin());
    std_msgs::ColorRGBA slam_color;
    slam_color.r = 0;
    slam_color.g = 1;
    slam_color.b = 0;
    slam_color.a = 1;
    slam_marker_traj_.colors.push_back(slam_color);
    if (slam_marker_traj_.colors.size() > 500)
      slam_marker_traj_.colors.erase(slam_marker_traj_.colors.begin());
    pub_slam_marker_.publish(slam_marker_traj_);

    slam_poses_.poses.push_back(slam_pose.pose);
    if (slam_poses_.poses.size() > 500)
      slam_poses_.poses.erase(slam_poses_.poses.begin());
    slam_poses_.header.stamp.fromSec(t_slam_);
    slam_poses_.header.frame_id = "map";
    pub_slam_pose_.publish(slam_poses_);
  }
}

void Top::filter_callback(const ros::TimerEvent &)
{
  if (!have_inited_gps_ or !have_inited_vel_)
  {
    ROS_INFO("wait for initial");
    return;
  }

  t_output_ = get_time_now();
  if (t_output_ - t_vel_ > 0.2)
  {
    ROS_INFO("no velocity input");
    return;
  }

  if (t_output_ - t_slam_ > 0.3)
  {
    // ROS_INFO("no slam input");
    command_no_gps_ = false;
  }

  Eigen::VectorXd X;
  if (fabs(curr_vel_) > 0.01)
    X = fusion_.readX(t_output_, true);
  else
    X = fusion_.readX(t_output_, false);
  MapFrame output_pos(localization_region_num_, X(0), X(1));
  // double output_yaw = heading_fusion_.readHeading(t_output_, true);
  double output_yaw = X(2);
  output_yaw_ = output_yaw;
  double t_pub = get_time_now();

  sensor_msgs::NavSatFix output_gps;
  output_gps = output_pos.MapFrame2GPS();

  geometry_msgs::Pose output_pose;
  char zone;
  LLtoUTM(output_gps.latitude, output_gps.longitude, output_pose.position.y,
          output_pose.position.x, &zone, GLOBAL_ZERO_X_, GLOBAL_ZERO_Y_);
  output_pose.position.z = 0;
  output_pose.orientation = tf::createQuaternionMsgFromYaw(output_yaw);

  // ROS_INFO("lat %f, lon %f, heading %f", output_gps.latitude,
  // output_gps.longitude, output_yaw);

  if (output_gps.latitude < 1 || output_gps.latitude > 90 ||
      isnan(output_gps.latitude) || isinf(output_gps.latitude))
  {
    ROS_INFO("output latitude invalid");
    return;
  }
  if (output_gps.longitude < 1 || output_gps.longitude > 180 ||
      isnan(output_gps.longitude) || isinf(output_gps.longitude))
  {
    ROS_INFO("output longitude invalid");
    return;
  }
  if (output_yaw < -M_PI || output_yaw > M_PI || isnan(output_yaw) ||
      isinf(output_yaw))
  {
    ROS_INFO("output heading invalid");
    return;
  }

  //定位失效手动恢复(支持误报使用)
  nh_priv_.param<bool>("manual_reset", manual_reset, false);
  if (manual_reset == true)
  {
    localization_fail_cnt_ = 0;
    nh_priv_.setParam("manual_reset", false);
  }
  if (enable_manual_reset && (localization_fail_cnt_ > LOCALIZATION_FAIL_NUM_))
  {
    ROS_ERROR("localization fail");
    return;
  }

  cyber_msgs::LocalizationEstimate localization_estimation;
  localization_estimation.header.frame_id = "world";
  localization_estimation.header.stamp.fromSec(t_pub);
  localization_estimation.status = 1;
  localization_estimation.latitude = output_gps.latitude;
  localization_estimation.longitude = output_gps.longitude;
  localization_estimation.altitude = 0;
  localization_estimation.pose = output_pose;
  localization_estimation.velocity.linear.x = curr_vel_;
  localization_estimation.velocity.linear.y = 0;
  localization_estimation.velocity.linear.z = 0;
  localization_estimation.velocity.angular = angular_velocity_;
  localization_estimation.acceleration.linear.x = 0;
  localization_estimation.acceleration.linear.y = 0;
  localization_estimation.acceleration.linear.z = 0;
  localization_estimation.acceleration.angular.x = 0;
  localization_estimation.acceleration.angular.y = 0;
  localization_estimation.acceleration.angular.z = 0;
  pub_localization_estimation_.publish(localization_estimation);

  if (localization_debug_)
  {
    double temp_yaw = output_yaw;
    // if (output_yaw < 0) temp_yaw += 2 * M_PI;
    std_msgs::Float64 yaw_angle;
    yaw_angle.data = temp_yaw / M_PI * 180;
    pub_localization_estimation_angle_.publish(yaw_angle);

    tf::Transform transform_filter;
    tf::Quaternion q_filter;
    transform_filter.setOrigin(tf::Vector3(output_pose.position.x,
                                           output_pose.position.y,
                                           output_pose.position.z));
    q_filter.setRPY(0, 0, output_yaw);
    transform_filter.setRotation(q_filter);
    br_filter_.sendTransform(tf::StampedTransform(
        transform_filter, ros::Time::now(), "map", "filter"));

    geometry_msgs::Point filter_marker = output_pose.position;
    filter_marker_traj_.header.frame_id = "map";
    filter_marker_traj_.header.stamp.fromSec(t_pub);
    filter_marker_traj_.ns = "filter";
    filter_marker_traj_.id = 102;
    filter_marker_traj_.type = 6;
    filter_marker_traj_.action = visualization_msgs::Marker::ADD;
    filter_marker_traj_.scale.x = 0.02;
    filter_marker_traj_.scale.y = 0.02;
    filter_marker_traj_.scale.z = 0.02;
    filter_marker_traj_.points.push_back(filter_marker);
    if (filter_marker_traj_.points.size() > 500)
      filter_marker_traj_.points.erase(filter_marker_traj_.points.begin());
    std_msgs::ColorRGBA filter_color;
    filter_color.r = 0;
    filter_color.g = 0;
    filter_color.b = 1;
    filter_color.a = 1;
    filter_marker_traj_.colors.push_back(filter_color);
    if (filter_marker_traj_.colors.size() > 500)
      filter_marker_traj_.colors.erase(filter_marker_traj_.colors.begin());
    pub_filter_marker_.publish(filter_marker_traj_);

    filter_poses_.poses.push_back(output_pose);
    if (filter_poses_.poses.size() > 500)
      filter_poses_.poses.erase(filter_poses_.poses.begin());
    filter_poses_.header.stamp.fromSec(t_pub);
    filter_poses_.header.frame_id = "map";
    pub_filter_pose_.publish(filter_poses_);
  }
}
