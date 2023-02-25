#include "near_safe.h"

void DELTA::Reset() {
  t = 0;
  //为了从上一帧坐标系下的速度推算出当前帧坐标系下的速度，q暂时也不能置单位四元数
  // q.setIdentity();
  p.setZero();
  //速度属于帧间连续性质变量，在Reset时不清零，如果当前存在速度反馈则可以赋值，否则维持当前状态
  // v.setZero();
}

Node::Node() : nh_("~") {
  // topic input
  nh_.param<std::string>("in_point_cloud_topic", in_point_cloud_topic_,
                         "/driver/pandar/point_cloud");
  nh_.param<std::string>("in_imu_topic", in_imu_topic_, "/driver/imu");
  nh_.param<std::string>("in_speed_topic", in_speed_topic_,
                         "/e100/speed_feedback");
  // topic output
  nh_.param<std::string>("out_emergency_topic", out_emergency_topic_,
                         "/perception/emergency");
  nh_.param<bool>("debug", debug_, false);
  // lidar_preprocess params
  nh_.param("lidar_preprocess/near_roi/near_noise_x_min",
            lp_params_.near_noise_x_min, lp_params_.near_noise_x_min);
  nh_.param("lidar_preprocess/near_roi/near_noise_x_max",
            lp_params_.near_noise_x_max, lp_params_.near_noise_x_max);
  nh_.param("lidar_preprocess/near_roi/near_noise_y_min",
            lp_params_.near_noise_y_min, lp_params_.near_noise_y_min);
  nh_.param("lidar_preprocess/near_roi/near_noise_y_max",
            lp_params_.near_noise_y_max, lp_params_.near_noise_y_max);
  nh_.param("lidar_preprocess/far_roi/livox_roi_x_min",
            lp_params_.livox_roi_x_min, lp_params_.livox_roi_x_min);
  nh_.param("lidar_preprocess/far_roi/livox_roi_x_max",
            lp_params_.livox_roi_x_max, lp_params_.livox_roi_x_max);
  nh_.param("lidar_preprocess/far_roi/livox_roi_y_min",
            lp_params_.livox_roi_y_min, lp_params_.livox_roi_y_min);
  nh_.param("lidar_preprocess/far_roi/livox_roi_y_max",
            lp_params_.livox_roi_y_max, lp_params_.livox_roi_y_max);
  nh_.param("lidar_preprocess/z_roi/livox_roi_z_min",
            lp_params_.livox_roi_z_min, lp_params_.livox_roi_z_min);
  nh_.param("lidar_preprocess/z_roi/livox_roi_z_max",
            lp_params_.livox_roi_z_max, lp_params_.livox_roi_z_max);
  nh_.param("lidar_preprocess/near_noise_filter/livox_noise_flag",
            lp_params_.livox_noise_flag, lp_params_.livox_noise_flag);
  nh_.param("lidar_preprocess/near_noise_filter/roi/livox_noise_x_min",
            lp_params_.livox_noise_x_min, lp_params_.livox_noise_x_min);
  nh_.param("lidar_preprocess/near_noise_filter/roi/livox_noise_x_max",
            lp_params_.livox_noise_x_max, lp_params_.livox_noise_x_max);
  nh_.param("lidar_preprocess/near_noise_filter/roi/livox_noise_y_min",
            lp_params_.livox_noise_y_min, lp_params_.livox_noise_y_min);
  nh_.param("lidar_preprocess/near_noise_filter/roi/livox_noise_y_max",
            lp_params_.livox_noise_y_max, lp_params_.livox_noise_y_max);
  nh_.param("lidar_preprocess/near_noise_filter/rate/livox_noise_rate",
            lp_params_.livox_noise_rate, lp_params_.livox_noise_rate);
  nh_.param("lidar_preprocess/near_noise_filter/min_direct_remove_intensity",
            lp_params_.min_direct_remove_intensity,
            lp_params_.min_direct_remove_intensity);
  nh_.param("lidar_preprocess/trailer_noise/trailer_x_min",
            lp_params_.trailer_x_min, lp_params_.trailer_x_min);
  nh_.param("lidar_preprocess/trailer_noise/trailer_y_min",
            lp_params_.trailer_y_min, lp_params_.trailer_y_min);
  nh_.param("lidar_preprocess/trailer_noise/trailer_y_max",
            lp_params_.trailer_y_max, lp_params_.trailer_y_max);
  nh_.param("lidar_preprocess/outlier_remove/min_outlier_intensity",
            lp_params_.min_outlier_intensity, lp_params_.min_outlier_intensity);
  nh_.param("lidar_preprocess/outlier_remove/search_radius",
            lp_params_.search_radius, lp_params_.search_radius);
  nh_.param("lidar_preprocess/outlier_remove/min_neighbors",
            lp_params_.min_neighbors, lp_params_.min_neighbors);
  // ground segmentation params
  nh_.param("ground_segmentation/n_bins", lp_params_.gs_params.n_bins,
            lp_params_.gs_params.n_bins);
  nh_.param("ground_segmentation/n_segments", lp_params_.gs_params.n_segments,
            lp_params_.gs_params.n_segments);
  nh_.param("ground_segmentation/gamma_rate", lp_params_.gs_params.gamma_rate,
            lp_params_.gs_params.gamma_rate);
  nh_.param("ground_segmentation/prior_ground_z",
            lp_params_.gs_params.prior_ground_z,
            lp_params_.gs_params.prior_ground_z);
  nh_.param("ground_segmentation/max_initial_slope",
            lp_params_.gs_params.max_initial_slope,
            lp_params_.gs_params.max_initial_slope);
  nh_.param("ground_segmentation/max_slope", lp_params_.gs_params.max_slope,
            lp_params_.gs_params.max_slope);
  nh_.param("ground_segmentation/line_search_segment_num",
            lp_params_.gs_params.line_search_segment_num,
            lp_params_.gs_params.line_search_segment_num);
  nh_.param("ground_segmentation/max_dist_to_line",
            lp_params_.gs_params.max_dist_to_line,
            lp_params_.gs_params.max_dist_to_line);
  // grid map params
  nh_.param("grid_map/roi_map/min_x", gm_params_.roi_params.min_x,
            gm_params_.roi_params.min_x);
  nh_.param("grid_map/roi_map/max_x", gm_params_.roi_params.max_x,
            gm_params_.roi_params.max_x);
  nh_.param("grid_map/roi_map/min_y", gm_params_.roi_params.min_y,
            gm_params_.roi_params.min_y);
  nh_.param("grid_map/roi_map/max_y", gm_params_.roi_params.max_y,
            gm_params_.roi_params.max_y);
  nh_.param("grid_map/roi_map/pixel_scale", gm_params_.roi_params.pixel_scale,
            gm_params_.roi_params.pixel_scale);
  nh_.param("near_safe_area/merge_count", frame_num_, frame_num_);

  nh_.param("near_safe_area/slow_down/x", gm_params_.safe_params.slow_down_x_,
            gm_params_.safe_params.slow_down_x_);
  nh_.param("near_safe_area/slow_down/y", gm_params_.safe_params.slow_down_y_,
            gm_params_.safe_params.slow_down_y_);
  nh_.param("near_safe_area/emergency_stop/x", gm_params_.safe_params.stop_x_,
            gm_params_.safe_params.stop_x_);
  nh_.param("near_safe_area/emergency_stop/y", gm_params_.safe_params.stop_y_,
            gm_params_.safe_params.stop_y_);
  nh_.param("near_safe_area/car_info/x", gm_params_.safe_params.car_x_,
            gm_params_.safe_params.car_x_);
  nh_.param("near_safe_area/car_info/y", gm_params_.safe_params.car_y_,
            gm_params_.safe_params.car_y_);

  // subscribe
  sub_imu_ = nh_.subscribe(in_imu_topic_, 1, &Node::ImuCallback, this);
  sub_pl2_ =
      nh_.subscribe(in_point_cloud_topic_, 1, &Node::CloudCallback, this);
  sub_vel_ = nh_.subscribe(in_speed_topic_, 1, &Node::VelCallback, this);

  // publish
  pub_emergency_ = nh_.advertise<std_msgs::Int8>(out_emergency_topic_, 1);
  if (debug_) {
    pub_merge_points_ =
        nh_.advertise<PointTypeCloud>("/perception/near_safe/merge_points", 1);
    image_transport::ImageTransport it(nh_);
    pub_roi_map_ = it.advertise("/perception/near_safe/roi_map", 1);
  }

  final_cloud_ = boost::make_shared<PointTypeCloud>();
  bg_.setZero();
  ba_.setZero();
  last_w.setZero();

  // lidar_preprocess
  lidar_preprocess_ = std::make_shared<LidarPreprocess>(lp_params_);

  // grid_map
  grid_map_ = std::make_shared<GridMap>(gm_params_);
}

void Node::CloudCallback(const PointTypeCloudConstPtr &in_cloud_ptr) {
  double top_receive_time = ros::Time::now().toSec();
  Frame new_frame;
  lidar_preprocess_->LivoxPreprocess2(*in_cloud_ptr, *new_frame.cloud_ptr_);
  new_frame.t = cur_delta_.t;
  new_frame.state.block<3, 3>(0, 0) = cur_delta_.q.matrix();
  new_frame.state.block<3, 1>(0, 3) = cur_delta_.p;
  new_frame.state.row(3) << 0, 0, 0,
      1;  //将DELTA中的四元数和平移向量转化为齐次变换矩阵

  cur_delta_.Reset();
  have_init_delta_ = false;

  if (frames_.size() == (frame_num_ + 1)) frames_.pop_front();

  for (auto i = frames_.begin(); i < frames_.end(); i++)
    (*i).state =
        (*i).state *
        new_frame.state;  //将当前列表内所有变换矩阵变为关于最新帧的变换

  frames_.push_back(new_frame);
  Merge();
  cv::Mat roi_map;
  int safe;
  grid_map_->GenerateGridMap(*final_cloud_, roi_map, safe);

  ROS_INFO("porcess time: %f", ros::Time::now().toSec() - top_receive_time);
  if (safe == 1) {
    ROS_INFO("slow_down");
  } else if (safe == 2) {
    ROS_INFO("emergency_stop");
  }

  std_msgs::Int8 safe_msg;
  safe_msg.data = safe;
  pub_emergency_.publish(safe_msg);
  if (debug_) {
    final_cloud_->header = in_cloud_ptr->header;
    pub_merge_points_.publish(final_cloud_);
    sensor_msgs::ImagePtr image_msg =
        cv_bridge::CvImage(std_msgs::Header(), "mono8", roi_map).toImageMsg();
    pcl_conversions::fromPCL(in_cloud_ptr->header.stamp,
                             image_msg->header.stamp);
    image_msg->header.frame_id = in_cloud_ptr->header.frame_id;
    pub_roi_map_.publish(image_msg);
  }
}

void Node::ImuCallback(const sensor_msgs::ImuConstPtr &input) {
  sensor_msgs::Imu imu_msg = *input;
  IMUData imu_data;
  float time_stamp = imu_msg.header.stamp.toSec();

  if (!have_init_delta_) {
    cur_delta_.t = time_stamp;
    if (cur_vel_ != 0) {
      //使用简化的汽车模型，认为每一帧开始积分时速度都是沿着x轴方向的
      cur_delta_.v(0) =
          cur_vel_;  //每过一个点云帧就重新开始积分计算以减少加速度计漂移造成的影响，特别是通过当前的速度反馈来矫正速度，如果没有速度反馈就在当前速度基础上开始积分
      cur_delta_.v(1) = 0;
      cur_delta_.v(2) = 0;
      //别忘了把q置单位四元数
      cur_delta_.q.setIdentity();
    } else {
      //如果不使用简化汽车模型，就需要考虑沿y轴的速度，将当前积分得到的，在前一帧点云坐标系下的速度投影到新坐标系下
      Eigen::Matrix3d vel_mat = cur_delta_.q.matrix().transpose();
      cur_delta_.v = vel_mat * cur_delta_.v;
      //别忘了把q置单位四元数
      cur_delta_.q.setIdentity();
    }
    have_init_delta_ = true;
    return;
  }

  imu_data.t = time_stamp;

  imu_data.a << imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y,
      imu_msg.linear_acceleration.z;

  imu_data.w << imu_msg.angular_velocity.x, imu_msg.angular_velocity.y,
      imu_msg.angular_velocity.z;

  if (last_w(0) == 0) {
    last_w = imu_data.w;
  } else {
    imu_data.w = (imu_data.w + last_w) / 2;
    last_w << imu_msg.angular_velocity.x, imu_msg.angular_velocity.y,
        imu_msg.angular_velocity.z;
  }

  Integrate2D(imu_data, bg_, ba_);
}

void Node::VelCallback(const cyber_msgs::VehicleSpeedFeedbackConstPtr &vel_in) {
  float vel = (float)(vel_in->speed_cmps) / 100.0;
  cur_vel_ = vel;
}

void Node::Merge() {
  final_cloud_->clear();
  for (auto i = frames_.begin(); i < (frames_.end() - 1); i++) {
    PointTypeCloudPtr temp_cloud(new PointTypeCloud);
    Eigen::Matrix4d inv;
    inv = InvHomo((*(i + 1)).state);
    Eigen::Matrix3d rotation_mat = inv.topLeftCorner(3, 3);
    Eigen::Vector3d euler = rotation_mat.eulerAngles(2, 1, 0);
    mrpt::poses::CPose3D pose(inv(0, 3), inv(1, 3), inv(2, 3), euler(0),
                              euler(1), euler(2));
    common::TransformPointCloud(*((*i).cloud_ptr_), pose, *temp_cloud);
    // pcl::transformPointCloud(*((*i).cloud_ptr_), *temp_cloud, inv);
    final_cloud_->insert(final_cloud_->end(), temp_cloud->begin(),
                         temp_cloud->end());
  }
  auto i = frames_.end() - 1;
  final_cloud_->insert(final_cloud_->end(), ((*i).cloud_ptr_)->begin(),
                       ((*i).cloud_ptr_)->end());
}

Eigen::Quaterniond Node::Expmap(const Eigen::Vector3d &w) {
  Eigen::AngleAxisd aa(w.norm(), w.normalized());
  Eigen::Quaterniond q;
  q = aa;
  return q;
}

Eigen::Matrix4d Node::InvHomo(Eigen::Matrix4d &in)  //给齐次变换矩阵求逆
{
  Eigen::Matrix4d inv;
  Eigen::Matrix3d temp = in.topLeftCorner(3, 3).transpose();
  inv.block<3, 1>(0, 3) = -temp * in.block<3, 1>(0, 3);
  inv.topLeftCorner(3, 3) = temp;
  inv.row(3) << 0, 0, 0, 1;
  return inv;
}

void Node::Integrate2D(const IMUData &data, const Eigen::Vector3d &bg,
                       const Eigen::Vector3d &ba) {
  Eigen::Vector3d w = data.w - bg;
  Eigen::Vector3d a = data.a - ba;

  //考虑到车载情况，对Z轴加速度和XY轴角速度进行忽略，这样也免去了重力补偿的必要
  w.head(2) << 0, 0;
  a.tail(1) << 0;
  //因为e300的IMU是朝下安装的
  w = -w;

  a(1) = -a(1);  //因为IMU系是左手系

  float dt = data.t - cur_delta_.t;
  //根据预积分公式对位姿进行积分计算
  cur_delta_.t = data.t;
  cur_delta_.p =
      cur_delta_.p + dt * cur_delta_.v +
      0.5 * dt * dt * (cur_delta_.q * a);  //计算当前在帧起始坐标系下的
  cur_delta_.v = cur_delta_.v +
                 dt * (cur_delta_.q * a);  //计算当前在帧起始坐标系下的速度投影
  cur_delta_.q = (cur_delta_.q * Expmap(w * dt)).normalized();  //计算旋转矩阵
}

int main(int argc, char **argv) {
  // 只在生成发布版的时候才解注释，否则在自己电脑上无法调试
  // if(!checkUSBKey()) return 0;

  ros::init(argc, argv, "near_safe");
  Node node;

  ros::spin();

  return 0;
}
