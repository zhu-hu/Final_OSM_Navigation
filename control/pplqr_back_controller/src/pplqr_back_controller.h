#ifndef PPLQR_BACK_CONTROLLER_H
#define PPLQR_BACK_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int8.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "Eigen/Dense"
#include "cyber_msgs/LinkAngle.h"
#include "cyber_msgs/LocalTrajList.h"
#include "cyber_msgs/LocalTrajPoint.h"
#include "cyber_msgs/LocalizationEstimate.h"
#include "cyber_msgs/TractorTrailerState.h"
#include "cyber_msgs/VehicleSpeedFeedback.h"
#include "cyber_msgs/VehicleSteerFeedback.h"
#include "linear_quadratic_regulator.h"

// ros时间戳同步的头文件,用来同步定位信息，前轮转角，拖挂角度三者的反馈数据
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include "common/log.h"

#define EPSILON 1e-5

namespace controller {

class PPLqrBackController {
 public:
  PPLqrBackController(ros::NodeHandle* nh);
  ~PPLqrBackController();

 private:
  struct TrajPoint {
    geometry_msgs::Pose point;
    int mode;
    TrajPoint(geometry_msgs::Pose point_in, int mode_in)
        : point(point_in), mode(mode_in) {}
  };

  struct PIDStruct {
    double Kp;
    double Ki;
    double Kd;
    double error_sum;
    double max_error_sum;
    double anti_i_saturation_coeff;
  };

  ros::NodeHandle* nh_;

  typedef message_filters::sync_policies::ApproximateTime<
      cyber_msgs::LocalizationEstimate, cyber_msgs::VehicleSteerFeedback,
      cyber_msgs::LinkAngle>
      FeedbackPolicy;
  message_filters::Subscriber<cyber_msgs::LocalizationEstimate>*
      sub_localization_;
  message_filters::Subscriber<cyber_msgs::VehicleSteerFeedback>* sub_steer_;
  message_filters::Subscriber<cyber_msgs::LinkAngle>* sub_link_angle_;

  message_filters::Synchronizer<FeedbackPolicy>* feedback_sync_;

  ros::Subscriber sub_local_traj_;

  //某一段轨迹完成控制之后，发布结束的topic，便于轨迹下发模块发布下一段轨迹
  ros::Publisher pub_one_trajectory_finished_;

  //如果轨迹发布模块的所有轨迹都发布完成了，就会下发所有轨迹下发完成的指令
  ros::Subscriber sub_trajectories_done_;

  ros::Publisher pub_steer_cmd_;
  ros::Publisher pub_speed_cmd_;
  // For Test
  ros::Timer timer_;
  ros::Publisher pub_desired_beta_;

  ros::Publisher pub_rviz_nearest_point_;
  ros::Publisher pub_rviz_lookahead_point_;

  //牵引车的状态(x,y,theta,delta,v,beta)
  cyber_msgs::TractorTrailerState trailer_state_;

  //拖挂车的状态(x,y,theta,v)
  cyber_msgs::TractorTrailerState second_trailer_state_;
  std::vector<TrajPoint> trajectory_points_;

  bool traj_flag_ = false;
  bool feedback_flag_ = false;

  //完成所有轨迹控制的标志位
  bool all_trajectories_done_ = false;

  //根据纯跟踪和几何关系算出来的一个beta的预期值
  double desired_beta_ = 0.0;

  // beta绝对值的最大值
  double max_fabs_beta_ = 0.0;

  //前轮转角的预期值(平衡点)
  double desired_delta_ = 0.0;

  //预期的速度值
  double desired_v_ = -1.0;

  int target_point_index_ = 0;

  double last_time_ = 0.0;

  //获取desire_beta时的一个误差比例系数
  // beta_desire = beta_pp + Kp(beta_pp - beta_now)
  double Kp_ = 0.0;  // Kp_值越大，这个控制就更＂积极＂（到达稳态时间更短）
  double size_param_L1_ = 0.0;
  double size_param_L2_ = 0.0;
  double size_param_M1_ = 0.0;
  double angle_weight_ = 0.0;
  double steer_weight_ = 0.0;
  double speed_weight_ = 0.0;
  double tolerance_ = 0.01;
  int max_iteration_ = 200;
  double look_ahead_dis_ = 5.0;

  //对于输出的低通滤波参数
  double filter_param_ = 0.7;

  //方向盘最大转向角,单位：degree
  double max_wheel_degree_;
  //前轮转角最大值,单位：rad
  double max_steer_rad_;

  bool lqr_test_ = false;
  double fixed_desired_beta_ = 0.0;

  PIDStruct pid_;
  bool use_pid_ = false;

  //上一次输出的前轮转角值，单位：rad
  double last_delta_output_ = 0.0;

  // Lqr的QR矩阵
  Eigen::Matrix<double, 1, 1> Q_;
  Eigen::Matrix<double, 2, 2> R_;

  // Lqr的A,B矩阵
  Eigen::Matrix<double, 1, 1> A_;
  Eigen::Matrix<double, 1, 2> B_;

  Eigen::Matrix<double, 1, 1> X_;

  Eigen::MatrixXd K_;

  void FeedbackCallback(
      const cyber_msgs::LocalizationEstimateConstPtr& localization_in,
      const cyber_msgs::VehicleSteerFeedbackConstPtr& steer_in,
      const cyber_msgs::LinkAngleConstPtr& link_angle_degree_in);

  void LocalTrajCallback(const cyber_msgs::LocalTrajList::ConstPtr& msg);

  void AllTrajectoriesDoneCallback(const std_msgs::BoolConstPtr& msg);

  void TimerCallback(const ros::TimerEvent& evt);

  inline double NormalAngle(const double& angle) {
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) {
      a += (2.0 * M_PI);
    }
    return a - M_PI;
  }

  inline double Deg2Rad(const double& angle) { return (angle * M_PI / 180.0); }

  inline double Rad2Deg(const double& angle) { return (angle * 180.0 / M_PI); }

  inline double Sign(const double& value) {
    if (value >= 0.0)
      return 1.0;
    else
      return -1.0;
  }

  //获取拖挂trailer的状态(x,y,theta,v)
  void GetSecondTrailerState();

  //找到参考轨迹上离当前第二节trailer位置距离最近的点
  int GetNearestTrajPoint();

  void PurePursuit();

  void ComputeSteerCmd();

  void ComputeSteerCmdUsingPID();

  void ShowPointInRviz(const ros::Publisher& pub,
                       const geometry_msgs::Point& point,
                       const std::string& color);

  inline double LimitAmplitude(const double& min, const double& max,
                               const double& cmd) {
    double value = cmd;
    if (cmd > max) value = max;
    if (cmd < min) value = min;
    return value;
  }
};
}  // namespace controller

#endif