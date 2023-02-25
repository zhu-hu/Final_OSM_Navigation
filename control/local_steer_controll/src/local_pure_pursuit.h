#ifndef LOCAL_PURE_PURSUIT_H
#define LOCAL_PURE_PURSUIT_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int8.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>

#include "cyber_msgs/LocalTrajList.h"
#include "cyber_msgs/VehicleSpeedFeedback.h"
#include "cyber_msgs/VehicleSteerFeedback.h"

namespace controller {
class LocalPurePursuit {
 public:
  LocalPurePursuit();
  ~LocalPurePursuit();

 private:
  struct TrajPoint {
    geometry_msgs::Pose point;
    int mode;
    TrajPoint(geometry_msgs::Pose point_in, int mode_in)
        : point(point_in), mode(mode_in) {}
  };

  bool path_flag_;

  int current_point_index_;

  double current_vel_;        //当前线速度,m/s
  double current_vel_angle_;  //当前角速度,rad/s
  double current_steer_angle_;  //当前的前轮转角，rad，根据方向盘反馈算出来的

  double base_reference_dis_;  //基准的预描距离
  double former_steer_cmd_;    //上一次下发的前轮转角值

  double temp_steer_error_;  //上一次下发的前轮转角与实际前轮转角的偏差

  double final_steer_cmd_;  //最终下发的前轮转角值

  double min_ref_speed_ = 2.0;  // scale:m/s

  double K_ref_ = 0.5;  //确定预描距离时的速度比例系数

  double K_alpha_ = 1.0;

  //速度自动驾驶时的最大速度，单位：m/s
  double max_velocity_ = 3.0;
  //速度自动驾驶时的最小速度，单位：m/s
  double min_velocity_ = 1.0;

  //方向盘到速度的转换系数
  double K_speed_ = -0.07;

  double temp_steer_cmd_;  //利用纯跟踪几何关系算出来的前轮转向数据

  std_msgs::Float64 wheel_output_;  //发布到底层的转向数据信息

  double Kp_error_;      //前轮转角P控制系数
  double Ki_error_;      //前轮转角I控制系数
  double filter_param_;  //低通滤波参数

  double Kp_wheel_;  //前轮转角到方向盘转角的传动比（单位统一情况下的）
  double wheelbase_;  //车辆前后轴中心之间的距离
  int wheel_max_;     //单位：0.1 度
  int wheel_zero_;    //方向盘零位，向左为负，单位：0.1 度

  geometry_msgs::PoseStamped target_pose_;  //在轨迹上的控制目标点的姿态

  geometry_msgs::PoseArray target_path_;      //目标路径，用于可视化
  std::vector<TrajPoint> target_trajectory_;  //待跟踪的目标轨迹

  //--------定义订阅者和发布者-----------
  ros::NodeHandle nh;
  //发布转向指令到底盘
  ros::Publisher pub_steer_cmd_;
  //发布速度指令到底盘
  ros::Publisher pub_speed_cmd_;
  //发布目标轨迹，用于可视化，可省略
  ros::Publisher pub_target_path_;
  //发布预描控制点，用于可视化
  ros::Publisher pub_control_target_point_;

  //订阅局部轨迹信息，局部轨迹的坐标系是车辆坐标系
  ros::Subscriber sub_local_trajectory_;
  //订阅车辆的速度反馈信息
  ros::Subscriber sub_speed_feedback_;
  //订阅车辆的转向反馈信息
  ros::Subscriber sub_steer_feedback_;
  //订阅车辆的角速度反馈信息
  ros::Subscriber sub_angular_vel_;
  // ROS定时器
  ros::Timer timer_;

  //---------参数服务相关变量------------
  //   dynamic_reconfigure::Server<steer_controller::pursuit_paramConfig>
  //   dr_srv; dynamic_reconfigure::Server<
  //       steer_controller::pursuit_paramConfig>::CallbackType cb;

  //---------定义成员函数----------------
  //只需要订阅定位消息和生成的轨迹信息，定位数据中的线速度直接用的底层反馈的结果，定位中的角速度直接用的imu解算出来的结果
  void LocalTrajectoryCallback(
      const cyber_msgs::LocalTrajList::ConstPtr& path_in);

  //底层速度反馈的回调函数
  void SpeedFeedback(const cyber_msgs::VehicleSpeedFeedbackConstPtr& speed_in);

  //底层转向反馈的回调函数
  void SteerFeedback(const cyber_msgs::VehicleSteerFeedbackConstPtr& steer_in);

  void ImuFeedback(const geometry_msgs::Vector3StampedConstPtr& imu_in);

  //根据pure pursuit方法计算转向输出
  void ComputeSteerCmd();

  void ShowTargetPoint(const geometry_msgs::PoseStamped in_target_point);

  //定时器回调函数，用于调用控制线程
  void TimerCallback(const ros::TimerEvent&);
};
}  // namespace controller

#endif