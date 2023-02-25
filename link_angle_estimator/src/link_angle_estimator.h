#pragma once
#include <mrpt/poses/CPose3D.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <eigen3/Eigen/Dense>
#include <mutex>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <queue>
#include <vector>

// ros时间戳同步的头文件,用来同步速度和前轮转角的反馈,便于EKF的状态预测一步
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include "common/utils/log.h"
#include "common/utils/math_utils.h"
#include "cyber_msgs/LinkAngle.h"
#include "cyber_msgs/LocalizationEstimate.h"
#include "cyber_msgs/TractorTrailerState.h"
#include "cyber_msgs/VehicleSpeedFeedback.h"
#include "cyber_msgs/VehicleSteerFeedback.h"
#include "hdmap/src/math/Polygon.h"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

// #include <opencv/cv.h>
// #include <opencv/cvaux.h>
// #include <opencv2/core/types.hpp>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

using namespace cyber_msgs;
using namespace cyber_common;
using common::math::Polygon2d;
using common::math::Vec2d;

inline double Deg2Rad(const double& angle) { return angle / 180.0 * M_PI; }

struct MotionMsg {
  double velocity;  //车x径向(X轴)速度,单位m/s
  double steer;     //车的前轮转角
};

class LinkAngleEstimator {
 public:
  LinkAngleEstimator(ros::NodeHandle& nh);
  ~LinkAngleEstimator() = default;

 private:
  //车身相关尺寸，从参数服务器中加载
  double size_param_L1_ = 0.0;
  double size_param_L2_ = 0.0;
  double size_param_M1_ = 0.0;
  double size_param_W1_ = 0.0;
  double size_param_W2_ = 0.0;
  double size_param_K1_ = 0.0;
  double size_param_K2_ = 0.0;
  double size_param_K3_ = 0.0;
  double size_param_K4_ = 0.0;

  bool is_measurement_init_ = false;
  bool is_ekf_init_ = false;
  bool is_motion_msg_init_ = false;

  PointCloud point_cloud_;
  PointCloudPtr trailer_point_cloud_;
  pcl::ModelCoefficients::Ptr line_coeff_;  // (x-a)/d = (y-b)/e = (z-c)/f

  double measurement_;  //连接角度,rad
  MotionMsg motion_msg_;

  //融合模式
  int fusion_mode_ = 0;

  // point cloud filter parameters
  bool enable_nosie_filter_ = true;
  double intensity_threshold_ = 25;
  double distance_threshold_ = 5.0;
  double R1_ = 3.2;
  double R2_ = 4.0;
  double angle_threshold_ = 3 * M_PI / 4;
  double livox_noise_rate_ = 0.005;
  double voxel_size_ = 0.2;
  size_t point_cloud_merge_frame_num = 3;
  size_t point_could_merge_count_ = 0;
  std::queue<size_t> cloud_size_queue_;

  std::vector<cv::Point2f> scan_points_;

  double min_dis_ = 0.7;
  double max_dis_ = 1.1;
  double max_fabs_angle_ = 0.8;
  double zero_bias_angle_degree_ = 0.0;

  // EKF
  double dt_ = 0.1;
  Eigen::Matrix<double, 2, 2> Q_;
  Eigen::Matrix<double, 1, 1> R_;
  Eigen::Matrix<double, 1, 1> X_est_;  // [beta]
  Eigen::Matrix<double, 1, 1> P_est_;  // [beta]

  double last_time_ = 0.0;

  ros::Publisher pub_pointcloud_vis_;
  ros::Publisher pub_car_polygons_;
  ros::Subscriber sub_point_cloud_;
  ros::Subscriber sub_laser_scan_;
  ros::Publisher pub_line_;
  ros::Timer filter_timer_;

  //为了输出trailerstate而添加的内容
  ros::Subscriber sub_localization_;
  ros::Publisher pub_trailer_state_;
  bool link_angle_updated_ = false;
  bool steer_updated_ = false;
  double steer_rad_;
  double beta_rad_;

  // For offline test
  bool fusion_offline_test_ = false;
  ros::Subscriber sub_measure_result_;
  ros::Publisher pub_result_test_;

  //融合过程中，回调函数的线程锁
  std::mutex ekf_mutex_;

  //铰接角度输出,单位:度(degeree)
  ros::Publisher pub_laser_measure_;
  ros::Publisher pub_livox_measure_;
  ros::Publisher pub_state_estimation_;  //融合输出

  typedef message_filters::sync_policies::ApproximateTime<
      cyber_msgs::VehicleSpeedFeedback, cyber_msgs::VehicleSteerFeedback>
      MotionPolicy;
  message_filters::Subscriber<cyber_msgs::VehicleSpeedFeedback>* sub_velocity_;
  message_filters::Subscriber<cyber_msgs::VehicleSteerFeedback>* sub_steer_;
  message_filters::Synchronizer<MotionPolicy>* motion_sync_;

  // callbacks
  void MotionCallback(const VehicleSpeedFeedbackConstPtr& velocity_in,
                      const VehicleSteerFeedbackConstPtr& steer_in);
  void PointCloudCallback(const PointCloudConstPtr& point_cloud_in);
  void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  void LocalizationCallback(
      const cyber_msgs::LocalizationEstimateConstPtr& localization_in);

  // For offline test
  void TestCallback(const std_msgs::Float64::ConstPtr& msg);
  // process the input point cloud, filter and transform it into world
  // coordinates
  void ProcessPointCloud(const PointCloudConstPtr& point_cloud_in);
  bool ClusterProcess(const PointCloud& point_could_in);
  bool RanSacFitProcess(const PointCloudPtr& point_could_in,
                        pcl::ModelCoefficients::Ptr& coeff,
                        pcl::PointIndices::Ptr& inliers);

  // ekf process
  void FilterCallback(const ros::TimerEvent&);
  const Eigen::Matrix<double, 1, 1> PredictMotion(
      const Eigen::Matrix<double, 1, 1>& X,
      const Eigen::Matrix<double, 2, 1>& U, const double dt);
  const Eigen::Matrix<double, 1, 1> Measurement(
      const Eigen::Matrix<double, 1, 1>& X);
  //获取状态量的雅克比矩阵
  const Eigen::Matrix<double, 1, 1> GetJF(const Eigen::Matrix<double, 1, 1>& X,
                                          const Eigen::Matrix<double, 2, 1>& U,
                                          const double dt);
  const Eigen::Matrix<double, 1, 1> GetJH();
  //获取预测误差更新时的雅克比矩阵
  const Eigen::Matrix<double, 1, 2> GetJG(const Eigen::Matrix<double, 1, 1>& X,
                                          const Eigen::Matrix<double, 2, 1>& U,
                                          const double dt);

  // utils
  void ShowPointCloud(const PointCloud& point_could_in);
  void ShowBoundingBox(const TractorTrailerState& tt_state);
  void ShowBoundingBox(const double beta);
  const Polygon2d GeneratePolygon(const double x, const double y,
                                  const double yaw, const double L,
                                  const double W, const double f,
                                  const double r);
  const Polygon2d GeneratePolygon(const double x, const double y,
                                  const double yaw, const double length,
                                  const double width);
};
