#ifndef DSTAR_DSTAR_H
#define DSTAR_DSTAR_H

#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
// For ompl::msg::setLogLevel
#include "ompl/util/Console.h"

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/SimpleSetup.h>
// #include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
//#include <ompl/geometric/planners/rrt/SORRTstar.h>

// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <opencv/cv.h>
#include <std_msgs/Float32.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <string>

#include "cyber_msgs/LocalTrajList.h"
#include "dstar_map/WayPoint.h"
#include "geometry_msgs/PointStamped.h"
#include "log.h"
#include "ros_transform.h"
#include "sensor_msgs/Image.h"
#include "tf/transform_broadcaster.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

constexpr double kCellSize = 0.2;

namespace huyao_rrt {

class RRT {
 public:
  RRT(ros::NodeHandle *nh, tf::TransformListener *listener, double runTime,
      double range);

  ~RRT(){};

 public:
  bool Plan(std::vector<double> start_in, std::vector<double> goal_in,
            bool reverse);

  bool Plan(std::vector<double> goal_in, bool reverse);

  bool Plancore(std::vector<double> start_in, std::vector<double> goal_in,
                bool reverse);  // default: runTime = 0.05, reverse = false

 private:
  void InitRRT();

  void GridMapCallback(const sensor_msgs::ImageConstPtr &map_in);
  void GoalCallback(const geometry_msgs::PoseStampedPtr &goal);
  void DirectionCallback(const std_msgs::Float32 &direction);
  void StartCallback(const geometry_msgs::PoseStampedPtr &start);
  void TimerCallback(const ros::TimerEvent &event);

 private:
  ros::NodeHandle *nh_;
  image_transport::ImageTransport *it_;
  image_transport::Subscriber subGridMap;
  ros::Publisher trajectory_pub;
  ros::Subscriber goalRRTChooseSub_;
  ros::Subscriber directionSub_;
  ros::Subscriber startLocalTrajSub_;
  ros::Timer timer_;

  geometry_msgs::PoseStamped goal_world_;
  geometry_msgs::PoseStamped start_world_;
  og::SimpleSetupPtr ss_;
  ob::StateSpacePtr space_;
  tf::TransformListener *listener_;
  double run_time_;
  cv::Mat localMap_;
  cv::Mat localMapWider_;
  nav_msgs::Path temp_path_;
  double range_;

  bool bGotGoal_;
  bool bGotStart_;
  bool bReverse_;
  bool bStartfromCar_;

  int timer_count_;
};

class DubinsMotionTestValidator : public ob::MotionValidator {
 public:
  DubinsMotionTestValidator(ob::SpaceInformation *si) : MotionValidator(si) {
    defaultSettings();
  }
  DubinsMotionTestValidator(const ob::SpaceInformationPtr &si)
      : MotionValidator(si) {
    defaultSettings();
  }
  ~DubinsMotionTestValidator() override = default;
  bool checkMotion(const ob::State *s1, const ob::State *s2) const override;
  bool checkMotion(const ob::State *s1, const ob::State *s2,
                   std::pair<ob::State *, double> &lastValid) const override;

 private:
  ob::DubinsStateSpace *stateSpace_;
  void defaultSettings();
};

inline double dist2Points(geometry_msgs::PoseStamped lhs,
                          geometry_msgs::PoseStamped rhs) {
  double x_diff = lhs.pose.position.x - rhs.pose.position.x;
  double y_diff = lhs.pose.position.x - rhs.pose.position.x;
  double distance = sqrt(x_diff * x_diff + y_diff * y_diff);
  return distance;
}

}  // namespace huyao_rrt

#endif  // DSTAR_DSTAR_H
