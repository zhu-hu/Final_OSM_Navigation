//
// Created by huyao on 18-8-24.
//

#ifndef PROJECT_RRT_CHOOSE_H
#define PROJECT_RRT_CHOOSE_H

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Path.h>
#include <opencv/cv.h>
#include <tf/transform_listener.h>

#include <opencv2/highgui/highgui.hpp>

#include "cyber_msgs/LocalTrajList.h"
#include "cyber_msgs/LocalTrajPoint.h"
#include "dstar_map/WayPoint.h"
#include "dstar_map/WayPointList.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "log.h"
#include "math_utils.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "ros_transform.h"
#include "sensor_msgs/Image.h"
#include "task_param.h"
#include "vec2d.h"

namespace huyao_RRT {

class RRTChoose {
 public:
  RRTChoose(ros::NodeHandle *nh, tf::TransformListener *listener);
  virtual ~RRTChoose();

 private:
  void GridMapCallback(const sensor_msgs::ImageConstPtr &map_in);

 public:
  void publishBestPath();

  void path1Callback(const nav_msgs::PathConstPtr &path_in);
  void path2Callback(const nav_msgs::PathConstPtr &path_in);
  void path3Callback(const nav_msgs::PathConstPtr &path_in);
  void path4Callback(const nav_msgs::PathConstPtr &path_in);
  void path5Callback(const nav_msgs::PathConstPtr &path_in);
  void path6Callback(const nav_msgs::PathConstPtr &path_in);

  void goalCallback(const geometry_msgs::PoseStampedPtr &goal);

  bool run();

 private:
  ros::NodeHandle *nh_;
  ros::Publisher trajectory_pub;
  ros::Publisher local_path_pub;
  ros::Publisher rrt_goal_pub;

  ros::Subscriber goalSub_;
  ros::Timer time_;

  ros::Subscriber path1Sub_;
  ros::Subscriber path2Sub_;
  ros::Subscriber path3Sub_;
  ros::Subscriber path4Sub_;
  ros::Subscriber path5Sub_;
  ros::Subscriber path6Sub_;

  cv::Mat localMap_;

  std::map<double, nav_msgs::Path::ConstPtr> pathMap_;

  bool findTargetMapPoint(const geometry_msgs::PoseStamped &point_in);
  bool findTargetMapPointReverse(const geometry_msgs::PoseStamped &point_in);

  void DecideCarDirection(const geometry_msgs::PoseStamped &point_in);
  void TimerCallback(const ros::TimerEvent &event);

  image_transport::ImageTransport *iT_;
  image_transport::Subscriber subGridMap_;

  cyber_msgs::LocalTrajList trajectory_;  // local path generated is stored
                                          // here.
  geometry_msgs::PoseStamped goal_world_;
  tf::TransformListener *listener_;

  ros::Time publishTime_;
  bool bPublish_;
  bool bGotGoal_;
  bool bReverse_;

  int timer_count_;
};

}  // namespace huyao_RRT

#endif  // PROJECT_RRT_CHOOSE_H
