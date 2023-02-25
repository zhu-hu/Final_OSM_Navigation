#include "link_angle_estimator.h"

LinkAngleEstimator::LinkAngleEstimator(ros::NodeHandle &nh) {
  X_est_(0, 0) = 0.0;
  P_est_(0, 0) = 1.0;
  // init publisher and subscriber
  pub_state_estimation_ =
      nh.advertise<cyber_msgs::LinkAngle>("/fusion_angle_degree", 10);

  pub_laser_measure_ =
      nh.advertise<std_msgs::Float64>("/laser_measure_angle_degree", 10);
  pub_livox_measure_ =
      nh.advertise<std_msgs::Float64>("/livox_measure_angle_degree", 10);

  nh.param("fusion_offline_test", fusion_offline_test_, false);

  if (fusion_offline_test_) {
    sub_measure_result_ = nh.subscribe("/laser_measure_angle_degree", 5,
                                       &LinkAngleEstimator::TestCallback, this);
    pub_result_test_ =
        nh.advertise<std_msgs::Float64>("/laser_measure_angle_degree_test", 10);
  }

  pub_pointcloud_vis_ =
      nh.advertise<visualization_msgs::Marker>("/pointcloud_vis", 10);
  pub_car_polygons_ = nh.advertise<visualization_msgs::MarkerArray>(
      "/tractortrailer_polygons", 10);

  sub_localization_ =
      nh.subscribe("/localization/estimation", 5,
                   &LinkAngleEstimator::LocalizationCallback, this);

  pub_trailer_state_ = nh.advertise<cyber_msgs::TractorTrailerState>(
      "/tractortrailer_state", 10);

  sub_velocity_ =
      new message_filters::Subscriber<cyber_msgs::VehicleSpeedFeedback>(
          nh, "/e100/speed_feedback", 1);
  sub_steer_ =
      new message_filters::Subscriber<cyber_msgs::VehicleSteerFeedback>(
          nh, "/e100/steer_feedback", 1);

  motion_sync_ = new message_filters::Synchronizer<MotionPolicy>(
      MotionPolicy(10), *sub_velocity_, *sub_steer_);
  motion_sync_->registerCallback(
      boost::bind(&LinkAngleEstimator::MotionCallback, this, _1, _2));

  if (fusion_mode_ == 2) {
    sub_point_cloud_ =
        nh.subscribe("/driver/livox/point_cloud", 10,
                     &LinkAngleEstimator::PointCloudCallback, this);
  }
  sub_laser_scan_ =
      nh.subscribe("/scan", 10, &LinkAngleEstimator::LaserScanCallback, this);

  pub_line_ =
      nh.advertise<visualization_msgs::Marker>("/scan_fit_line", 5, this);
  // filter_timer_ = nh.createTimer(ros::Duration(dt_),
  //                                &LinkAngleEstimator::FilterCallback, this);

  nh.param("SizeParam_L1", size_param_L1_, 1.62);
  nh.param("SizeParam_L2", size_param_L2_, 2.04);
  nh.param("SizeParam_M1", size_param_M1_, 0.57);
  nh.param("SizeParam_W1", size_param_W1_, 1.56);
  nh.param("SizeParam_W2", size_param_W2_, 0.9);
  nh.param("SizeParam_K1", size_param_K1_, 0.33);
  nh.param("SizeParam_K2", size_param_K2_, 0.58);
  nh.param("SizeParam_K3", size_param_K3_, 0.50);
  nh.param("SizeParam_K4", size_param_K4_, 0.14);

  nh.param("min_dis", min_dis_, 0.7);
  nh.param("max_dis", max_dis_, 1.1);
  nh.param("max_fabs_angle", max_fabs_angle_, 0.8);
  nh.param("zero_bias_angle_degree", zero_bias_angle_degree_, 0.0);

  double steer_error, vel_error, measure_error_deg;
  nh.param("fusion_mode", fusion_mode_, 0);
  nh.param("steer_error", steer_error, 0.1);
  nh.param("vel_error", vel_error, 0.1);
  nh.param("measure_error", measure_error_deg, 2.0);

  // init ekf parameters
  //预测误差
  Eigen::Matrix<double, 2, 1> Qv(
      steer_error, vel_error);  //(steer_error,vel_error) [rad,m/s]
  Q_ = Qv.asDiagonal();
  Q_ = Q_ * Q_;

  //观测误差
  Eigen::Matrix<double, 1, 1> Rv(Deg2Rad(measure_error_deg));
  R_ = Rv.asDiagonal();
  R_ = R_ * R_;

  motion_msg_.velocity = 0.0;
  motion_msg_.steer = 0.0;
}

void LinkAngleEstimator::MotionCallback(
    const VehicleSpeedFeedbackConstPtr &velocity_in,
    const VehicleSteerFeedbackConstPtr &steer_in) {
  if (!is_motion_msg_init_) is_motion_msg_init_ = true;
  if (!steer_updated_) steer_updated_ = true;
  motion_msg_.velocity = velocity_in->speed_cmps / 100.0;
  motion_msg_.steer = -steer_in->steer_0p1d / 10.0 * M_PI / 180.0 / 16.0;

  steer_rad_ = motion_msg_.steer;

  //如果不是融合模式，不需要再进行接下来的操作了
  if (!(fusion_mode_ == 1 || fusion_mode_ == 2)) return;

  if (!is_ekf_init_) {
    last_time_ = ros::Time::now().toSec();
    AINFO << "ekf not init!!!";
    return;
  }

  ekf_mutex_.lock();
  double delta_t = ros::Time::now().toSec() - last_time_;

  // Prediction
  Eigen::Matrix<double, 2, 1> U(motion_msg_.steer, motion_msg_.velocity);
  Eigen::Matrix<double, 1, 1> X_pred = PredictMotion(X_est_, U, delta_t);
  const auto JF = GetJF(X_est_, U, delta_t);
  Eigen::MatrixXd JG(1, 2);
  JG = GetJG(X_est_, U, delta_t);
  P_est_ = JF * P_est_ * JF.transpose() + JG * Q_ * JG.transpose();
  X_est_ = X_pred;

  cyber_msgs::LinkAngle link_angle_msg;
  link_angle_msg.header.stamp = ros::Time::now();
  link_angle_msg.link_angle_degree = X_est_(0, 0) * 180.0 / M_PI;
  if (!link_angle_updated_) link_angle_updated_ = true;
  beta_rad_ = X_est_(0, 0);
  pub_state_estimation_.publish(link_angle_msg);

  // AINFO << "Motion Prediction !!! ";
  // AINFO << "X_est_ : " << X_est_ << " rad";
  // AINFO << "P_est_ : " << P_est_;

  double beta_rad = X_est_(0, 0);
  ShowBoundingBox(beta_rad);

  last_time_ = ros::Time::now().toSec();

  ekf_mutex_.unlock();
}

void LinkAngleEstimator::PointCloudCallback(
    const PointCloudConstPtr &point_cloud_in) {
  CHECK_NOTNULL(point_cloud_in);

  // 1. pre-process the point cloud
  ProcessPointCloud(point_cloud_in);

  // 2. cluster the local point cloud
  ClusterProcess(point_cloud_);

  // 3. fitting point cloud cluster
  line_coeff_.reset(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_(new pcl::PointIndices);
  if (!RanSacFitProcess(trailer_point_cloud_, line_coeff_, inliers_)) {
    AERROR << "Error: Failed to fit the first trailer";
    return;
  }

  double k = line_coeff_->values[4] / line_coeff_->values[3];
  double phi = atan2(1 / k, 1.0);

  //发布livox雷达测量的角度
  std_msgs::Float64 angle_msg;
  angle_msg.data = phi * 180.0 / M_PI;
  pub_livox_measure_.publish(angle_msg);

  AINFO << "livox measure : " << angle_msg.data << " Deg";

  if (fusion_mode_ == 2) {
    measurement_ = phi;
    if (!is_measurement_init_) is_measurement_init_ = true;
    ADEBUG << "Update measurements";

    if (!is_ekf_init_) {
      X_est_(0, 0) = measurement_;
      is_ekf_init_ = true;
      return;
    }

    ekf_mutex_.lock();
    Eigen::Matrix<double, 1, 1> Y(measurement_);
    // Measurement
    const auto JH = GetJH();
    const auto Y_pred = Measurement(X_est_);

    // Update
    const auto E = JH * P_est_ * JH.transpose();
    const auto K = P_est_ * JH.transpose() * (R_ + E).inverse();
    X_est_ = X_est_ + K * (Y - Y_pred);
    P_est_ = (Eigen::Matrix<double, 1, 1>::Identity() - K * JH) * P_est_;

    AINFO << "Livox Update !!! ";
    AINFO << "X_est_ : " << X_est_ << " rad";
    AINFO << "P_est_ : " << P_est_;

    ekf_mutex_.unlock();
  }
}

void LinkAngleEstimator::LaserScanCallback(
    const sensor_msgs::LaserScan::ConstPtr &msg) {
  scan_points_.clear();
  std::cout << "laser_point_nums : " << msg->ranges.size() << std::endl;
  double angle_min = msg->angle_min;
  double angle_max = msg->angle_max;
  double angle_increment = msg->angle_increment;
  for (int i = 0; i < msg->ranges.size(); i++) {
    //距离滤波
    if (msg->ranges[i] < min_dis_ || msg->ranges[i] > max_dis_ ||
        std::isnan(msg->ranges[i]))
      continue;
    double angle = angle_min + angle_increment * i;
    //角度滤波
    if (fabs(angle) > max_fabs_angle_) continue;
    cv::Point2f point;
    point.x = msg->ranges[i] * cos(angle);  // scale:m
    point.y = msg->ranges[i] * sin(angle);  // scale:m
    // std::cout << "x : " << point.x << "  y : " << point.y
    //           << " range : " << msg->ranges[i] << std::endl;
    scan_points_.emplace_back(point);
  }

  std::cout << "scan_point_nums : " << scan_points_.size() << std::endl;

  cv::Vec4f line_pare;

  cv::fitLine(scan_points_, line_pare, 2, 0, 1e-2, 1e-2);

  std::cout << "line pare : " << line_pare << std::endl;
  geometry_msgs::Point point0;
  point0.x = line_pare[2];
  point0.y = line_pare[3];

  double k = line_pare[1] / line_pare[0];
  double theta = 0.0;
  if (k > 0.0) {  //拖挂车向左偏为正
    theta = M_PI_2 - atan2(k, 1.0);
  } else {
    theta = -M_PI_2 - atan2(k, 1.0);  //拖挂车向右偏为负
  }
  // std::cout << "theta : " << theta << " Rad" << std::endl;
  std::cout << "theta original : " << theta * 180.0 / M_PI << " Deg"
            << std::endl;

  //发布laser的测量值
  //运动模型中，beta = theta1 - theta2,拖挂车向左偏为正，拖挂车向右偏为负
  std_msgs::Float64 angle_msg;
  angle_msg.data = theta * 180.0 / M_PI - zero_bias_angle_degree_;
  pub_laser_measure_.publish(angle_msg);

  AINFO << "laser measure : " << angle_msg.data << " Deg";

  if (fusion_mode_ == 0) {
    cyber_msgs::LinkAngle link_angle_msg;
    link_angle_msg.header.stamp = ros::Time::now();
    link_angle_msg.link_angle_degree = angle_msg.data;

    if (!link_angle_updated_) link_angle_updated_ = true;
    beta_rad_ = angle_msg.data * M_PI / 180.0;
    pub_state_estimation_.publish(link_angle_msg);
  } else if (fusion_mode_ == 1) {
    measurement_ = Deg2Rad(theta * 180.0 / M_PI - zero_bias_angle_degree_);
    if (!is_measurement_init_) is_measurement_init_ = true;
    ADEBUG << "Update measurements";

    if (!is_ekf_init_) {
      X_est_(0, 0) = measurement_;
      is_ekf_init_ = true;
      return;
    }

    ekf_mutex_.lock();
    Eigen::Matrix<double, 1, 1> Y(measurement_);
    // Measurement
    const auto JH = GetJH();
    const auto Y_pred = Measurement(X_est_);

    // Update
    const auto E = JH * P_est_ * JH.transpose();
    const auto K = P_est_ * JH.transpose() * (R_ + E).inverse();
    X_est_ = X_est_ + K * (Y - Y_pred);
    P_est_ = (Eigen::Matrix<double, 1, 1>::Identity() - K * JH) * P_est_;

    AINFO << "Laser Update !!! ";
    AINFO << "X_est_ : " << X_est_ << " rad";
    AINFO << "P_est_ : " << P_est_;

    ekf_mutex_.unlock();
  }

  geometry_msgs::Point point1, point2;
  point1.x = scan_points_[0].x;
  point1.y = k * (point1.x - point0.x) + point0.y;
  point2.x = scan_points_.back().x;
  point2.y = k * (point2.x - point0.x) + point0.y;

  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = "laser";
  line_marker.header.stamp = ros::Time::now();
  line_marker.id = 0;
  line_marker.lifetime = ros::Duration(0);
  line_marker.type = visualization_msgs::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::Marker::ADD;

  line_marker.scale.x = 0.1;
  line_marker.scale.y = 0.1;
  line_marker.scale.z = 0.1;
  line_marker.color.a = 1.0;
  line_marker.color.r = 1.0;
  line_marker.color.g = 0.0;
  line_marker.color.b = 0.0;
  geometry_msgs::Point point;
  point.x = point1.x;
  point.y = point1.y;
  line_marker.points.emplace_back(point);
  point.x = point2.x;
  point.y = point2.y;
  line_marker.points.emplace_back(point);
  pub_line_.publish(line_marker);
}

void LinkAngleEstimator::LocalizationCallback(
    const cyber_msgs::LocalizationEstimateConstPtr &localization_in) {
  if (link_angle_updated_ == false || steer_updated_ == false) return;
  cyber_msgs::TractorTrailerState state_msg;
  state_msg.x = localization_in->pose.position.x;
  state_msg.y = localization_in->pose.position.y;
  state_msg.velocity = localization_in->velocity.linear.x;
  state_msg.theta = tf::getYaw(localization_in->pose.orientation);
  state_msg.steer = steer_rad_;
  state_msg.beta = beta_rad_;
  pub_trailer_state_.publish(state_msg);
}

void LinkAngleEstimator::TestCallback(const std_msgs::Float64::ConstPtr &msg) {
  std_msgs::Float64 pub_data;
  pub_data.data = msg->data;
  pub_result_test_.publish(pub_data);

  measurement_ = Deg2Rad(msg->data);
  if (!is_measurement_init_) is_measurement_init_ = true;
  ADEBUG << "Update measurements";

  if (!is_ekf_init_) {
    X_est_(0, 0) = measurement_;
    is_ekf_init_ = true;
    return;
  }

  Eigen::Matrix<double, 1, 1> Y(measurement_);
  // Measurement
  const auto JH = GetJH();
  const auto Y_pred = Measurement(X_est_);

  // Update
  AINFO << "P Updated !!! ";
  AINFO << "last P_est_ : " << P_est_;
  const auto E = JH * P_est_ * JH.transpose();
  const auto K = P_est_ * JH.transpose() * (R_ + E).inverse();
  X_est_ = X_est_ + K * (Y - Y_pred);
  P_est_ = (Eigen::Matrix<double, 1, 1>::Identity() - K * JH) * P_est_;
  AINFO << "E : " << E;
  AINFO << "K : " << K;
  AINFO << "X_est_ ; " << X_est_;
  AINFO << "P_est_ : " << P_est_;
}

void LinkAngleEstimator::ProcessPointCloud(
    const PointCloudConstPtr &point_cloud_in) {
  const auto raw_point_cloud_size = point_cloud_in->size();
  const auto &raw_point_cloud = point_cloud_in->points;
  ADEBUG << "In point cloud size: " << raw_point_cloud_size;

  std::vector<int> indices;
  std::vector<int> final_indices;
  for (size_t i = 0; i < raw_point_cloud_size; ++i) {
    // position filter
    if (raw_point_cloud[i].x < -5.0 || raw_point_cloud[i].x > -1.0) continue;
    if (raw_point_cloud[i].y < -5.0 || raw_point_cloud[i].y > 5.0) continue;
    if (raw_point_cloud[i].z < 1.4 || raw_point_cloud[i].z > 1.8) continue;

    // distance filter
    const double dis = hypot(raw_point_cloud[i].x, raw_point_cloud[i].y);
    if (dis > distance_threshold_) continue;

    // angle filter
    const double point_theta =
        atan2(raw_point_cloud[i].y, raw_point_cloud[i].x);
    if ((point_theta > 0.0 && point_theta < angle_threshold_) ||
        (point_theta < 0.0 && point_theta > -angle_threshold_))
      continue;

    // noise filter first step
    if (enable_nosie_filter_) {
      if (i == 0 || i == raw_point_cloud_size - 1) continue;
      const auto &pre_pt = raw_point_cloud[i - 1];
      const auto &cur_pt = raw_point_cloud[i];
      const auto &next_pt = raw_point_cloud[i + 1];
      const double pre_dis = hypot(pre_pt.y - cur_pt.y, pre_pt.x - cur_pt.x);
      const double next_dis = hypot(next_pt.y - cur_pt.y, next_pt.x - cur_pt.x);
      const double threshold = dis * livox_noise_rate_;
      if (pre_dis > threshold || next_dis > threshold) continue;
    }
    indices.emplace_back(i);
  }

  for (size_t i = 0; i < indices.size(); i++) {
    // noise filter second step
    if (enable_nosie_filter_) {
      if (i == 0 || i == indices.size() - 1) continue;
      const auto &pre_pt = raw_point_cloud[indices[i - 1]];
      const auto &cur_pt = raw_point_cloud[indices[i]];
      const auto &next_pt = raw_point_cloud[indices[i + 1]];
      const double pre_dis = hypot(pre_pt.y - cur_pt.y, pre_pt.x - cur_pt.x);
      const double next_dis = hypot(next_pt.y - cur_pt.y, next_pt.x - cur_pt.x);
      const double dis = hypot(cur_pt.y, cur_pt.x);
      const double threshold = dis * livox_noise_rate_;
      if (pre_dis > threshold || next_dis > threshold) continue;
    }
    final_indices.emplace_back(indices[i]);
  }

  // copy from final_indices
  PointCloudPtr filter_cloud_ptr(new PointCloud);
  filter_cloud_ptr->resize(final_indices.size());
  for (size_t i = 0; i < final_indices.size(); ++i) {
    filter_cloud_ptr->points[i] = raw_point_cloud[final_indices[i]];
  }
  if (filter_cloud_ptr->empty()) {
    AERROR << "ERROR: Failed to filter input point cloud!!!";
    return;
  }

  // voxel grid filter
  PointCloudPtr voxel_grid_cloud_ptr(new PointCloud);
  pcl::VoxelGrid<PointType> voxel_grid;
  voxel_grid.setInputCloud(filter_cloud_ptr);
  voxel_grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  voxel_grid.filter(*voxel_grid_cloud_ptr);

  // merge point cloud
  point_could_merge_count_++;

  if (point_could_merge_count_ > point_cloud_merge_frame_num) {
    auto first_cloud_size = cloud_size_queue_.front();
    cloud_size_queue_.pop();
    point_cloud_.erase(point_cloud_.begin(),
                       point_cloud_.begin() + first_cloud_size);
  }
  auto size = voxel_grid_cloud_ptr->size() + point_cloud_.size();
  point_cloud_.insert(point_cloud_.end(), voxel_grid_cloud_ptr->begin(),
                      voxel_grid_cloud_ptr->end());
  point_cloud_.resize(size);
  cloud_size_queue_.push(voxel_grid_cloud_ptr->size());
  ADEBUG << "Current point cloud size: " << size;

  ShowPointCloud(point_cloud_);

  if (point_could_merge_count_ >= 10 * point_cloud_merge_frame_num)
    point_could_merge_count_ = point_cloud_merge_frame_num;
}

bool LinkAngleEstimator::ClusterProcess(const PointCloud &point_could_in) {
  trailer_point_cloud_.reset(new PointCloud);
  for (const auto &point : point_could_in.points) {
    // intesity threshold
    if (point.intensity <= intensity_threshold_) continue;

    double dis = hypot(point.x, point.y);
    if (dis > R1_ && dis < R2_) {
      trailer_point_cloud_->points.emplace_back(point);
      trailer_point_cloud_->points.back().z = 0.0;
    }
  }
  ADEBUG << "trailer point size is: " << trailer_point_cloud_->points.size();

  return !trailer_point_cloud_->points.empty();
}

bool LinkAngleEstimator::RanSacFitProcess(const PointCloudPtr &point_could_in,
                                          pcl::ModelCoefficients::Ptr &coeff,
                                          pcl::PointIndices::Ptr &inliers) {
  pcl::SACSegmentation<PointType> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_LINE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.2);
  seg.setInputCloud(point_could_in);
  seg.segment(*inliers, *coeff);

  return !inliers->indices.empty();
}

void LinkAngleEstimator::FilterCallback(const ros::TimerEvent &) {
  if (!is_motion_msg_init_) {
    AINFO << "Wating for vehicle state feedback...";
    return;
  }
  if (!is_measurement_init_) {
    AINFO << "Wating for measurement...";
    return;
  }

  if (!is_ekf_init_) {
    X_est_(0, 0) = measurement_;
    is_ekf_init_ = true;
    return;
  }
  Eigen::Matrix<double, 1, 1> Y(measurement_);
  Eigen::Matrix<double, 2, 1> U(motion_msg_.steer, motion_msg_.velocity);

  // Prediction
  const auto X_pred = PredictMotion(X_est_, U, dt_);
  const auto JF = GetJF(X_est_, U, dt_);
  const auto JG = GetJG(X_est_, U, dt_);
  const auto P_pred = JF * P_est_ * JF.transpose() + JG * Q_ * JG.transpose();

  // Measurement
  const auto JH = GetJH();
  const auto Y_pred = Measurement(X_pred);

  // Update
  const auto E = JH * P_pred * JH.transpose();
  const auto K = P_pred * JH.transpose() * (R_ + E).inverse();
  X_est_ = X_pred + K * (Y - Y_pred);
  P_est_ = (Eigen::Matrix<double, 1, 1>::Identity() - K * JH) * P_pred;

  cyber_msgs::LinkAngle link_angle_msg;
  link_angle_msg.header.stamp = ros::Time::now();
  link_angle_msg.link_angle_degree = X_est_(0, 0) * 180.0 / M_PI;

  if (!link_angle_updated_) link_angle_updated_ = true;
  beta_rad_ = X_est_(0, 0);
  pub_state_estimation_.publish(link_angle_msg);
  // ShowBoundingBox(tt_state);
  ADEBUG << "EKF process finished...";
}

const Eigen::Matrix<double, 1, 1> LinkAngleEstimator::PredictMotion(
    const Eigen::Matrix<double, 1, 1> &X, const Eigen::Matrix<double, 2, 1> &U,
    const double dt) {
  const double L1 = size_param_L1_;
  const double L2 = size_param_L2_;
  const double M1 = size_param_M1_;
  const double beta_0 = X(0, 0);
  const double a = U[0];
  const double v = U[1];
  const Eigen::Matrix<double, 1, 1> beta(
      beta_0 + (v / (L1 * L2) *
                (L2 * tan(a) - L1 * sin(beta_0) + M1 * cos(beta_0) * tan(a))) *
                   dt);
  return beta;
}

const Eigen::Matrix<double, 1, 1> LinkAngleEstimator::Measurement(
    const Eigen::Matrix<double, 1, 1> &X) {
  const Eigen::Matrix<double, 1, 1> H(1.0);

  return H * X;
}

const Eigen::Matrix<double, 1, 1> LinkAngleEstimator::GetJF(
    const Eigen::Matrix<double, 1, 1> &X, const Eigen::Matrix<double, 2, 1> &U,
    const double dt) {
  Eigen::Matrix<double, 1, 1> JF;
  const double L1 = size_param_L1_;
  const double L2 = size_param_L2_;
  const double M1 = size_param_M1_;
  const double beta = X(0, 0);
  const double a = U[0];
  const double v = U[1];
  const double t3 =
      1 + v / (L1 * L2) *
              (L2 * tan(a) - L1 * cos(beta) - M1 * sin(beta) * tan(a)) * dt;

  JF << t3;

  return JF;
}

const Eigen::Matrix<double, 1, 1> LinkAngleEstimator::GetJH() {
  Eigen::Matrix<double, 1, 1> JH;
  JH << 1.0;

  return JH;
}

// parcial F/parcial U
const Eigen::Matrix<double, 1, 2> LinkAngleEstimator::GetJG(
    const Eigen::Matrix<double, 1, 1> &X, const Eigen::Matrix<double, 2, 1> &U,
    const double dt) {
  Eigen::MatrixXd JG(1, 2);
  double beta0 = X(0, 0);
  double a = U[0];
  double v = U[1];
  const double L1 = size_param_L1_;
  const double L2 = size_param_L2_;
  const double M1 = size_param_M1_;

  JG << v / (L1 * L2) *
            (L2 / (cos(a) * cos(a)) - L1 * sin(beta0) +
             M1 * cos(beta0) / (cos(a) * cos(a))) *
            dt,
      1.0 / (L1 * L2) *
          (L2 * tan(a) - L1 * sin(beta0) + M1 * cos(beta0) * tan(a)) * dt;
  return JG;
}

void LinkAngleEstimator::ShowPointCloud(const PointCloud &point_could_in) {
  visualization_msgs::Marker marker;
  int id = 0;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "base_link";
  marker.type = visualization_msgs::Marker::POINTS;
  // marker.lifetime = ros::Duration(0.2);
  marker.id = id;
  marker.ns = "estimator";

  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  for (const auto &p : point_could_in.points) {
    geometry_msgs::Point pt;
    pt.x = p.x;
    pt.y = p.y;
    pt.z = p.z;

    marker.points.emplace_back(pt);
  }

  double start_theta = 2 * M_PI / 3;
  while (start_theta < 4 * M_PI / 3) {
    geometry_msgs::Point pt;
    pt.y = R1_ * sin(start_theta);
    pt.x = R1_ * cos(start_theta);
    marker.points.emplace_back(pt);

    pt.y = R2_ * sin(start_theta);
    pt.x = R2_ * cos(start_theta);
    marker.points.emplace_back(pt);

    start_theta += 0.05;
  }

  pub_pointcloud_vis_.publish(marker);
}

const Polygon2d LinkAngleEstimator::GeneratePolygon(
    const double x, const double y, const double yaw, const double L,
    const double W, const double f, const double r) {
  Eigen::MatrixXd vertex(2, 4);
  vertex(0, 0) = -r;
  vertex(1, 0) = 0.5 * W;
  vertex(0, 1) = -r;
  vertex(1, 1) = -0.5 * W;
  vertex(0, 2) = L + f;
  vertex(1, 2) = -0.5 * W;
  vertex(0, 3) = L + f;
  vertex(1, 3) = 0.5 * W;

  Eigen::Matrix2d rot_matrix;
  rot_matrix << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
  vertex = rot_matrix * vertex;

  std::vector<common::math::Vec2d> points;
  for (int i = 0; i < 4; i++) {
    points.push_back(common::math::Vec2d(vertex(0, i) + x, vertex(1, i) + y));
  }

  return common::math::Polygon2d(points);
}

const Polygon2d LinkAngleEstimator::GeneratePolygon(const double x,
                                                    const double y,
                                                    const double yaw,
                                                    const double length,
                                                    const double width) {
  common::math::Vec2d ref_point1;
  ref_point1.set_x(x + length / 2.0 * cos(yaw));
  ref_point1.set_y(y + length / 2.0 * sin(yaw));

  common::math::Vec2d ref_point2;
  ref_point2.set_x(x - length / 2.0 * cos(yaw));
  ref_point2.set_y(y - length / 2.0 * sin(yaw));

  std::vector<common::math::Vec2d> points;
  points.push_back(
      common::math::Vec2d(ref_point1.x() + width / 2.0 * sin(yaw),
                          ref_point1.y() - width / 2.0 * cos(yaw)));
  points.push_back(
      common::math::Vec2d(ref_point1.x() - width / 2.0 * sin(yaw),
                          ref_point1.y() + width / 2.0 * cos(yaw)));
  points.push_back(
      common::math::Vec2d(ref_point2.x() - width / 2.0 * sin(yaw),
                          ref_point2.y() + width / 2.0 * cos(yaw)));
  points.push_back(
      common::math::Vec2d(ref_point2.x() + width / 2.0 * sin(yaw),
                          ref_point2.y() - width / 2.0 * cos(yaw)));
  return common::math::Polygon2d(points);
}

void LinkAngleEstimator::ShowBoundingBox(const TractorTrailerState &tt_state) {
  // Calculate trailer state
  const double x_tractor = tt_state.x;
  const double y_tractor = tt_state.y;
  const double theta_tractor = tt_state.theta;
  const double beta = tt_state.beta;

  const double L1 = size_param_L1_;
  const double L2 = size_param_L2_;
  const double M1 = size_param_M1_;

  const double theta_trailer = theta_tractor - beta;
  const double x_hitch = x_tractor - M1 * cos(theta_tractor);
  const double y_hitch = y_tractor - M1 * sin(theta_tractor);
  const double x_trailer = x_hitch - L2 * cos(theta_trailer);
  const double y_trailer = y_hitch - L2 * sin(theta_trailer);

  // Calculate bounding box
  std::vector<Polygon2d> polygons;
  auto polygon_tractor =
      GeneratePolygon(x_tractor, y_tractor, theta_tractor, L1, size_param_W1_,
                      size_param_K1_, size_param_K2_);
  auto polygon_trailer =
      GeneratePolygon(x_trailer, y_trailer, theta_trailer, L2, size_param_W2_,
                      -size_param_K3_, size_param_K4_);
  polygons.push_back(std::move(polygon_tractor));
  polygons.push_back(std::move(polygon_trailer));

  // std::cout << "In Visulizer::ShowCar" << std::endl;
  visualization_msgs::MarkerArray polygon_array;
  int id = 0;
  for (const auto &polygon : polygons) {
    visualization_msgs::Marker marker;
    marker.ns = "polygons";
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    //以绿色表示
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.r = 0.0;
    marker.color.a = 1.0;
    marker.id = id++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    for (const auto &pt : polygon.points()) {
      geometry_msgs::Point tmp;
      tmp.x = pt.x();
      tmp.y = pt.y();
      tmp.z = 0.0;
      marker.points.emplace_back(tmp);
    }
    geometry_msgs::Point front;
    front.x = polygon.points().front().x();
    front.y = polygon.points().front().y();
    marker.points.emplace_back(front);
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.lifetime = ros::Duration(0.5);
    polygon_array.markers.emplace_back(marker);
  }
  pub_car_polygons_.publish(polygon_array);
}

void LinkAngleEstimator::ShowBoundingBox(const double beta) {
  double tractor_length = size_param_L1_ + size_param_K1_ + size_param_K2_;
  double tractor_width = size_param_W1_;

  double trailer_length = size_param_L2_ + size_param_K4_ - size_param_K3_;
  double trailer_width = size_param_W2_;

  double trailer_center_x =
      size_param_M1_ - size_param_K1_ +
      (size_param_K3_ + trailer_length / 2.0) * cos(-beta);

  double trailer_center_y =
      (size_param_K3_ + trailer_length / 2.0) * sin(-beta);
  // Calculate bounding box
  std::vector<Polygon2d> polygons;
  auto polygon_tractor = GeneratePolygon(-tractor_length / 2.0, 0.0, 0.0,
                                         tractor_length, tractor_width);
  auto polygon_trailer = GeneratePolygon(trailer_center_x, trailer_center_y,
                                         -beta, trailer_length, trailer_width);
  polygons.push_back(std::move(polygon_tractor));
  polygons.push_back(std::move(polygon_trailer));

  // std::cout << "In Visulizer::ShowCar" << std::endl;
  visualization_msgs::MarkerArray polygon_array;
  int id = 0;

  //添加单线雷达的设备位置显示，以红色表示
  visualization_msgs::Marker laser_marker;
  laser_marker.ns = "polygons";
  laser_marker.header.frame_id = "laser";
  laser_marker.header.stamp = ros::Time::now();
  laser_marker.color.g = 0.0;
  laser_marker.color.b = 0.0;
  laser_marker.color.r = 1.0;
  laser_marker.color.a = 1.0;
  laser_marker.id = id++;
  laser_marker.type = visualization_msgs::Marker::CUBE;
  laser_marker.action = visualization_msgs::Marker::ADD;
  laser_marker.pose.position.x = 0.025;
  laser_marker.pose.position.y = 0.0;
  laser_marker.pose.position.z = 0.0;
  laser_marker.scale.x = 0.05;
  laser_marker.scale.y = 0.1;
  laser_marker.scale.z = 0.02;
  laser_marker.lifetime = ros::Duration(0.2);
  polygon_array.markers.emplace_back(laser_marker);

  //添加旋转轴的位置显示,以黄色表示
  laser_marker.ns = "polygons";
  laser_marker.header.frame_id = "laser";
  laser_marker.header.stamp = ros::Time::now();
  laser_marker.type = visualization_msgs::Marker::CYLINDER;
  laser_marker.id = id++;
  laser_marker.color.g = 1.0;
  laser_marker.color.b = 0.0;
  laser_marker.color.r = 1.0;
  laser_marker.color.a = 1.0;
  laser_marker.pose.position.x = size_param_M1_ - size_param_K1_;
  laser_marker.pose.position.y = 0.0;
  laser_marker.pose.position.z = 0.0;
  laser_marker.scale.x = 0.05;
  laser_marker.scale.y = 0.05;
  laser_marker.scale.z = 0.02;
  polygon_array.markers.emplace_back(laser_marker);

  //添加轴与拖挂车的连杆,以绿色表示
  visualization_msgs::Marker line_marker;
  line_marker.ns = "polygons";
  line_marker.header.frame_id = "laser";
  line_marker.header.stamp = ros::Time::now();
  line_marker.color.g = 1.0;
  line_marker.color.b = 0.0;
  line_marker.color.r = 0.0;
  line_marker.color.a = 1.0;
  line_marker.id = id++;
  line_marker.type = visualization_msgs::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::Marker::ADD;
  line_marker.scale.x = 0.02;
  line_marker.scale.y = 0.02;
  line_marker.scale.z = 0.02;
  geometry_msgs::Point tmp;
  tmp.x = size_param_M1_ - size_param_K1_;
  tmp.y = 0.0;
  tmp.z = 0.0;
  line_marker.points.emplace_back(tmp);
  tmp.x = size_param_M1_ - size_param_K1_ + size_param_K3_ * cos(-beta);
  tmp.y = size_param_K3_ * sin(-beta);
  tmp.z = 0.0;
  line_marker.points.emplace_back(tmp);
  line_marker.lifetime = ros::Duration(0.2);
  polygon_array.markers.emplace_back(line_marker);

  //添加轴与牵引车的连杆
  line_marker.id = id++;
  line_marker.points.clear();
  tmp.x = 0.0;
  tmp.y = 0.0;
  tmp.z = 0.0;
  line_marker.points.emplace_back(tmp);
  tmp.x = size_param_M1_ - size_param_K1_;
  tmp.y = 0.0;
  tmp.z = 0.0;
  line_marker.points.emplace_back(tmp);
  polygon_array.markers.emplace_back(line_marker);

  //添加Tractor和Trailer的文字显示
  visualization_msgs::Marker text_marker;
  text_marker.ns = "polygons";
  text_marker.header.frame_id = "laser";
  text_marker.header.stamp = ros::Time::now();
  text_marker.color.g = 0.0;
  text_marker.color.b = 0.0;
  text_marker.color.r = 1.0;
  text_marker.color.a = 1.0;
  text_marker.id = id++;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.text = std::string("Tractor");
  text_marker.pose.position.x = -tractor_length / 2.0;
  text_marker.pose.position.y = 0.0;
  text_marker.pose.position.z = 0.0;
  text_marker.scale.x = 0.5;
  text_marker.scale.y = 0.5;
  text_marker.scale.z = 0.4;
  text_marker.lifetime = ros::Duration(0.2);
  polygon_array.markers.emplace_back(text_marker);

  text_marker.id = id++;
  text_marker.text = std::string("Trailer");
  text_marker.pose.position.x = trailer_center_x;
  text_marker.pose.position.y = trailer_center_y;
  text_marker.pose.position.z = 0.0;
  polygon_array.markers.emplace_back(text_marker);

  //显示牵引车和拖挂车的外轮廓矩形
  for (const auto &polygon : polygons) {
    visualization_msgs::Marker marker;
    marker.ns = "polygons";
    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();

    //以绿色表示
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.r = 0.0;
    marker.color.a = 1.0;
    marker.id = id++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    for (const auto &pt : polygon.points()) {
      geometry_msgs::Point tmp;
      tmp.x = pt.x();
      tmp.y = pt.y();
      tmp.z = 0.0;
      marker.points.emplace_back(tmp);
    }
    geometry_msgs::Point front;
    front.x = polygon.points().front().x();
    front.y = polygon.points().front().y();
    marker.points.emplace_back(front);
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.lifetime = ros::Duration(0.2);
    polygon_array.markers.emplace_back(marker);
  }

  pub_car_polygons_.publish(polygon_array);
}