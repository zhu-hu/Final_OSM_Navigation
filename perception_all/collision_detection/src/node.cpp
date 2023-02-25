/**
 *node.cpp
 *brief:to Merge several frames of point cloud
 *author:Yang Chenglin
 *date:2021/05/20
 **/
#include "node.h"

void DELTA::Reset() {
  t = 0;
  //为了从上一帧坐标系下的速度推算出当前帧坐标系下的速度，q暂时也不能置单位四元数
  // q.setIdentity();
  p.setZero();
  //速度属于帧间连续性质变量，在Reset时不清零，如果当前存在速度反馈则可以赋值，否则维持当前状态
  // v.setZero();
}

bool MyPointCmpK(POINT a, POINT b) {
  float dist1 = a.x * a.x + a.y * a.y;
  float dist2 = b.x * b.x + b.y * b.y;
  return dist1 < dist2;
}

Node::Node() : nh_("~") {
  nh_.param<int>("merge_frame_num", merge_frame_num_, 5);
  nh_.param<float>("roi/x_min", roi_params_.x_min, 4.0);
  nh_.param<float>("roi/x_max", roi_params_.x_max, 5.0);
  nh_.param<float>("roi/y_min", roi_params_.y_min, -1.1);
  nh_.param<float>("roi/y_max", roi_params_.y_max, 1.1);
  nh_.param<float>("roi/z_max", roi_params_.z_max, 1.0);
  nh_.param("roi/overlap_area_remove/angle_up_right", lp_params_.angle_up_right,
            lp_params_.angle_up_right);
  nh_.param("roi/overlap_area_remove/angle_up_left", lp_params_.angle_up_left,
            lp_params_.angle_up_left);
  nh_.param("roi/overlap_area_remove/angle_down_right",
            lp_params_.angle_down_right, lp_params_.angle_down_right);
  nh_.param<float>("threshold/height", height_sh_, height_sh_);
  nh_.param<float>("threshold/dist", dist_sh_, dist_sh_);
  // nh_.param<float>("max_z_height", gm_params_.height_thres, 0.30);
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
  // // grid map params
  // nh_.param("grid_map/roi_map/min_x", gm_params_.roi_params.min_x,
  //           gm_params_.roi_params.min_x);
  // nh_.param("grid_map/roi_map/max_x", gm_params_.roi_params.max_x,
  //           gm_params_.roi_params.max_x);
  // nh_.param("grid_map/roi_map/min_y", gm_params_.roi_params.min_y,
  //           gm_params_.roi_params.min_y);
  // nh_.param("grid_map/roi_map/max_y", gm_params_.roi_params.max_y,
  //           gm_params_.roi_params.max_y);
  // nh_.param("max_z_map_pixel_scale", gm_params_.roi_params.pixel_scale,
  //           gm_params_.roi_params.pixel_scale);

  final_cloud_ = boost::make_shared<CLOUD>();
  bg_.setZero();
  ba_.setZero();
  last_w.setZero();

  sub_imu_ = nh_.subscribe("/driver/imu", 1, &Node::ImuCallback, this);
  sub_pl2_ =
      nh_.subscribe("/driver/livox/point_cloud", 1, &Node::CloudCallback, this);
  sub_vel_ = nh_.subscribe("/e100/speed_feedback", 1, &Node::VelCallback, this);
  pub_jump_points_ = nh_.advertise<geometry_msgs::PoseArray>(
      "/perception/edge_jump_points", 10);
  if (debug_) {
    pub_pl2_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/perception/road_edge", 10);
    pub_pl1_ = nh_.advertise<sensor_msgs::PointCloud2>("/perception/line", 10);
  }
  lidar_preprocesser_ = std::make_shared<LidarPreprocess>(lp_params_);
  // grid_map_ = std::make_shared<GridMap>(gm_params_);
}

Node::~Node() {}
void Node::CloudCallback(const CLOUD_CONSTPTR &input) {
  CLOUD_PTR cloud(new CLOUD);
  lidar_preprocesser_->CloudPreprocess(*input, *cloud);

  //检测路沿
  CLOUD_PTR pub_cloud(new CLOUD);
  CLOUD_PTR cloudinline(new CLOUD);
  std::vector<CLOUD_PTR> laser(180);
  for (int i = 0; i < laser.size(); i++)
    laser[i].reset(new CLOUD);

  // float height_sh = 0.25;
  // float dist_sh = 0.1;
  //按角度将点云分配到射线
  for (int i = 0; i < cloud->points.size(); i++) {
    float tan_point = cloud->points[i].x / cloud->points[i].y;
    float angle = atan(tan_point) * 180.0 / M_PI;
    if (angle < 0)
      angle += 180;
    int index = angle / 1;
    if (index < laser.size()) {
      laser[index]->points.push_back(cloud->points[i]);
    }
  }

  for (int k = 0; k < laser.size(); k++) {
    sort(laser[k]->points.begin(), laser[k]->points.end(),
         MyPointCmpK); //排序
    bool flag = 0;
    int index_max, index_min;
    float max_z, min_z, rad, dist;
    if (laser[k]->points.size() > 5) {
      //查找跳变点
      for (int j = 0; j < laser[k]->points.size() - 1; j++) {
        if (flag)
          break;
        for (int i = j + 1; i < laser[k]->points.size(); i++) {
          dist = sqrt((laser[k]->points[i].y - laser[k]->points[j].y) *
                          (laser[k]->points[i].y - laser[k]->points[j].y) +
                      (laser[k]->points[i].x - laser[k]->points[j].x) *
                          (laser[k]->points[i].x - laser[k]->points[j].x));

          if (laser[k]->points[i].z - laser[k]->points[j].z > height_sh_ &&
              dist < dist_sh_) {
            index_max = i;
            index_min = j;
            flag = 1;
            break;
          }
          if (dist > dist_sh_)
            break;
        }
      }
      if (flag)
        for (int m = index_min; m <= index_max; m++)
          cloudinline->points.push_back(laser[k]->points[m]);
    }
  }

  Frame new_frame;
  new_frame.cloud_ptr_->reserve(cloudinline->size());
  for (size_t i = 0; i < cloudinline->size(); i++) {
    const POINT &pt = cloudinline->at(i);
    if (pt.x > roi_params_.x_min && pt.x < roi_params_.x_max &&
        pt.y > roi_params_.y_min && pt.y < roi_params_.y_max &&
        pt.z < roi_params_.z_max) {
      new_frame.cloud_ptr_->push_back(pt);
    }
  }
  new_frame.t = cur_delta_.t;
  new_frame.state.block<3, 3>(0, 0) = cur_delta_.q.matrix();
  new_frame.state.block<3, 1>(0, 3) = cur_delta_.p;
  new_frame.state.row(3) << 0, 0, 0,
      1; //将DELTA中的四元数和平移向量转化为齐次变换矩阵

  cur_delta_.Reset();
  have_init_delta_ = false;

  if (frames_.size() == (merge_frame_num_ + 1))
    frames_.pop_front();

  for (auto i = frames_.begin(); i < frames_.end(); i++)
    (*i).state = (*i).state *
                 new_frame.state; //将当前列表内所有变换矩阵变为关于最新帧的变换

  frames_.emplace_back(new_frame);

  Merge();
  // cv::Mat max_height_map, min_height_map;
  // grid_map_->GenerateGridMap(*final_cloud_, max_height_map,
  //                            min_height_map);       //处理后的跳变点
  // grid_map_->UpdateGridMap(*cloud, max_height_map); //原始单帧点云

  geometry_msgs::PoseArray edge_point;
  edge_point.header.stamp = ros::Time::now();
  edge_point.poses.reserve(final_cloud_->size());
  for (size_t i = 0; i < final_cloud_->size(); i++) {
    // if (!grid_map_->isHeightObstacle(final_cloud_->points[i], max_height_map,
    //                                  min_height_map)) {
    geometry_msgs::Pose pose;
    pose.position.x = final_cloud_->points[i].x;
    pose.position.y = final_cloud_->points[i].y;
    edge_point.poses.emplace_back(pose);
    // }
  }
  pub_jump_points_.publish(edge_point);
  ROS_INFO(
      "[collision_detection]:edge point size: %zu,after height threshold: %zu",
      final_cloud_->size(), edge_point.poses.size());

  if (debug_) {
    pub_cloud->reserve(final_cloud_->size());
    for (size_t i = 0; i < final_cloud_->size(); i++) {
      // if (!grid_map_->isHeightObstacle(final_cloud_->points[i],
      // max_height_map,
      //                                  min_height_map)) {
      pub_cloud->push_back(final_cloud_->points[i]);
      // }
    }
    sensor_msgs::PointCloud2 msg_seg_pc;
    pcl::toROSMsg(*final_cloud_, msg_seg_pc);
    msg_seg_pc.header.frame_id = "livox_frame";
    pub_pl2_.publish(msg_seg_pc);

    sensor_msgs::PointCloud2 msg_seg_pc2;
    pcl::toROSMsg(*pub_cloud, msg_seg_pc2);
    msg_seg_pc2.header.frame_id = "livox_frame";
    pub_pl1_.publish(msg_seg_pc2);
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
          cur_vel_; //每过一个点云帧就重新开始积分计算以减少加速度计漂移造成的影响，特别是通过当前的速度反馈来矫正速度，如果没有速度反馈就在当前速度基础上开始积分
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

Eigen::Quaterniond Node::Expmap(const Eigen::Vector3d &w) {
  Eigen::AngleAxisd aa(w.norm(), w.normalized());
  Eigen::Quaterniond q;
  q = aa;
  return q;
}

Eigen::Matrix4d Node::InvHomo(Eigen::Matrix4d &in) //给齐次变换矩阵求逆
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

  a(1) = -a(1); //因为IMU系是左手系

  float dt = data.t - cur_delta_.t;
  //根据预积分公式对位姿进行积分计算
  cur_delta_.t = data.t;
  cur_delta_.p =
      cur_delta_.p + dt * cur_delta_.v +
      0.5 * dt * dt * (cur_delta_.q * a); //计算当前在帧起始坐标系下的
  cur_delta_.v = cur_delta_.v +
                 dt * (cur_delta_.q * a); //计算当前在帧起始坐标系下的速度投影
  cur_delta_.q = (cur_delta_.q * Expmap(w * dt)).normalized(); //计算旋转矩阵
}

void Node::Merge() {
  final_cloud_->clear();
  for (auto i = frames_.begin(); i < (frames_.end() - 1); i++) {
    CLOUD temp_cloud;
    Eigen::Matrix4d inv;
    inv = InvHomo((*(i + 1)).state);
    pcl::transformPointCloud(*((*i).cloud_ptr_), temp_cloud, inv);
    final_cloud_->insert(final_cloud_->end(), temp_cloud.begin(),
                         temp_cloud.end());
  }
  auto i = frames_.end() - 1;
  final_cloud_->insert(final_cloud_->end(), ((*i).cloud_ptr_)->begin(),
                       ((*i).cloud_ptr_)->end());
}

int main(int argc, char **argv) {
  // 只在生成发布版的时候才解注释，否则在自己电脑上无法调试
  printf("CollisionDetection module version: 1.1.1\n");
  // if (!checkUSBKey()) return 0;

  ros::init(argc, argv, "collision_detection");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  Node ins;
  ros::spin();
  return 0;
}
