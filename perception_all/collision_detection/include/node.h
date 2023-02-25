/**
 *node.h
 *brief:to Merge several frames of point cloud
 *author:Yang Chenglin
 *date:2021/05020
 **/

#ifndef NODE_H
#define NODE_H

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8.h>
#include <sys/types.h>
#include <termios.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <deque>
#include <iostream>
#include <string>
#include <vector>

#include "cyber_msgs/VehicleSpeedFeedback.h"
#include "grid_map.h"
#include "lidar_preprocess.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#define ACC_G 9.8

using POINT = pcl::PointXYZI;
using CLOUD = pcl::PointCloud<POINT>;
using CLOUD_PTR = CLOUD::Ptr;
using CLOUD_CONSTPTR = CLOUD::ConstPtr;

struct DELTA //保存帧间相对位姿结构体
{
  double t;             // 时间戳
  Eigen::Quaterniond q; // 姿态角
  Eigen::Vector3d p;    // 位移量
  Eigen::Vector3d v;    // 线速度
  void Reset();
  DELTA() {
    t = 0;
    q.setIdentity();
    p.setZero();
    v.setZero();
  }
};

struct IMUData {
  double t;          // 时间戳
  Eigen::Vector3d w; // 角速度
  Eigen::Vector3d a; // 线速度
};

struct ROI {
  float x_min = 4.0;
  float x_max = 5.0;
  float y_min = -1.1;
  float y_max = 1.1;
  float z_max = 1.0;
  float z_min = -0.2;
};

typedef struct {
  double a = 0.0, b = 0.0, c = 0.0, d = 0.0;
} model_t;

class Frame //将点云和其相对上一帧的位姿存储在一个类内
{
public:
  Frame() : cloud_ptr_(new CLOUD()) {}

  float t;               // 时间戳
  Eigen::Matrix4d state; // 姿态
  CLOUD_PTR cloud_ptr_;  // 局部坐标系点云
};

class Node {
public:
  Node();
  ~Node();

  void CloudCallback(const CLOUD_CONSTPTR &input);
  void ImuCallback(const sensor_msgs::ImuConstPtr &input);
  void VelCallback(const cyber_msgs::VehicleSpeedFeedbackConstPtr &vel_in);

private:
  ros::NodeHandle nh_;

  ros::Subscriber sub_imu_;
  ros::Subscriber sub_pl2_;
  ros::Subscriber sub_vel_;

  ros::Publisher pub_emergency_;
  ros::Publisher pub_merged_pointcloud_;
  ros::Publisher pub_ground_pointcloud_;

  ros::Publisher pub_pl1_;
  ros::Publisher pub_pl2_;
  ros::Publisher pub_jump_points_;

  ROI roi_params_;

  int merge_frame_num_ = 5; //执行多帧融合的帧数, default=5
  std::deque<Frame> frames_;

  bool debug_ = false;    // 是否输出叠加点云
  CLOUD_PTR final_cloud_; //用于发布融合后的最终点云

  bool have_init_delta_ = false; //判断是否有初始化Δ
  DELTA cur_delta_;
  float cur_vel_ = 0; //假设反馈的车辆速度方向与IMU的x轴方向相同
  Eigen::Vector3d bg_;    //角速度bias
  Eigen::Vector3d ba_;    //加速度bias
  Eigen::Vector3d last_w; //上一时刻的角速度，用于提升积分精度

  float height_sh_ = 0.25;
  float dist_sh_ = 0.1;
  std::deque<bool> collision_deque_;

  LidarPreprocessParams lp_params_;
  std::shared_ptr<LidarPreprocess> lidar_preprocesser_;

  // GridMap::GridMapParams gm_params_;
  // std::shared_ptr<GridMap> grid_map_;

private:
  Eigen::Quaterniond Expmap(const Eigen::Vector3d &w);
  Eigen::Matrix4d InvHomo(Eigen::Matrix4d &in);
  void Integrate2D(const IMUData &data, const Eigen::Vector3d &bg,
                   const Eigen::Vector3d &ba);
  //将当前类内全部的点云根据相对位姿进行融合叠加,最终结果是基于最后一帧坐标系的
  void Merge();
};

#endif // NODE_H
