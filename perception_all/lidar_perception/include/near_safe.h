#ifndef NEAR_SAFE_H
#define NEAR_SAFE_H

#include <cv_bridge/cv_bridge.h>
#include <cyber_msgs/VehicleSpeedFeedback.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <mrpt/poses/CPose3D.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int8.h>

#include <Eigen/Eigen>

#include "common/point_type.h"
#include "common/util.h"
#include "grid_map/grid_map.h"
#include "lidar_preprocess/lidar_preprocess.h"

struct DELTA  //保存帧间相对位姿结构体
{
  double t;              // 时间戳
  Eigen::Quaterniond q;  // 姿态角
  Eigen::Vector3d p;     // 位移量
  Eigen::Vector3d v;     // 线速度
  void Reset();
  DELTA() {
    t = 0;
    q.setIdentity();
    p.setZero();
    v.setZero();
  }
};

struct IMUData {
  double t;           // 时间戳
  Eigen::Vector3d w;  // 角速度
  Eigen::Vector3d a;  // 线速度
};

class Frame  //将点云和其相对上一帧的位姿存储在一个类内
{
 public:
  Frame() : cloud_ptr_(new PointTypeCloud()) {}

  float t;                       // 时间戳
  Eigen::Matrix4d state;         // 姿态
  PointTypeCloudPtr cloud_ptr_;  // 局部坐标系点云
};

class Node {
 public:
  Node();

  void CloudCallback(const PointTypeCloudConstPtr &in_cloud_ptr);
  void ImuCallback(const sensor_msgs::ImuConstPtr &input);
  void VelCallback(const cyber_msgs::VehicleSpeedFeedbackConstPtr &vel_in);

 private:
  ros::NodeHandle nh_;

  // subscriber
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_pl2_;
  ros::Subscriber sub_vel_;

  // topic name(subscriber)
  std::string in_point_cloud_topic_;
  std::string in_imu_topic_;
  std::string in_speed_topic_;

  // publisher
  ros::Publisher pub_emergency_;
  ros::Publisher pub_merge_points_;
  image_transport::Publisher pub_roi_map_;

  // topic name(publisher)
  std::string out_emergency_topic_;

  int frame_num_ = 5;  //执行多帧融合的帧数, default=5
  std::deque<Frame> frames_;
  PointTypeCloudPtr final_cloud_;  //融合后的最终点云

  bool have_init_delta_ = false;  //判断是否有初始化Δ
  DELTA cur_delta_;
  float cur_vel_ = 0;  //假设反馈的车辆速度方向与IMU的x轴方向相同
  Eigen::Vector3d bg_;     //角速度bias
  Eigen::Vector3d ba_;     //加速度bias
  Eigen::Vector3d last_w;  //上一时刻的角速度，用于提升积分精度

  // lidar_preprocess
  LidarPreprocess::LidarPreprocessParams lp_params_;
  std::shared_ptr<LidarPreprocess> lidar_preprocess_;

  // roi_map
  GridMap::GridMapParams gm_params_;
  std::shared_ptr<GridMap> grid_map_;

  // debug
  bool debug_ = false;

 private:
  Eigen::Quaterniond Expmap(const Eigen::Vector3d &w);
  Eigen::Matrix4d InvHomo(Eigen::Matrix4d &in);
  void Integrate2D(const IMUData &data, const Eigen::Vector3d &bg,
                   const Eigen::Vector3d &ba);
  //将当前类内全部的点云根据相对位姿进行融合叠加,最终结果是基于最后一帧坐标系的
  void Merge();
};

#endif  // NEAR_SAFE_H
