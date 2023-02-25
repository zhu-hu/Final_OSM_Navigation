#ifndef OSM_ROUTE_MANAGER_H
#define OSM_ROUTE_MANAGER_H

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>

#include "common/math/discrete_points_referenceline_smoother.h"
#include "cyber_msgs/LocalizationEstimate.h"
#include "osm_map_manager.h"
#include "osm_parser/src/path_finder_algorithm/dijkstra.h"
#include "parameter/tiggo_model.h"
namespace planning {
class OsmRouteManager {
 public:
  OsmRouteManager(ros::NodeHandle* nh, Parameter* param,
                  OsmMapManager* osm_map_manager);
  ~OsmRouteManager();

  bool OsmRoutePlanner(const cyber_msgs::LocalizationEstimate& adc_state);

  bool OsmRoutePlanner(int source_id, int target_id);

  void PublishRoutePath();

  void PubSmoothedRoutePath();

  inline OsmMapManager* osm_map_manager() const { return osm_map_manager_; }

  inline bool TargetIdUpdated() const { return target_id_updated_; }

  inline int CurrentNavigationNodeId() const {
    return current_navigation_node_id_;
  }

  inline geometry_msgs::PoseStamped LocalNavigationPose() const {
    return local_navigation_pose_;
  }

 private:
  Parameter* param_;
  ros::NodeHandle* nh_;
  ros::Publisher pub_route_path_;
  //订阅目的地
  ros::Subscriber sub_target_point_;

  ros::Subscriber sub_move_base_goal_;

  ros::Publisher pub_smoothed_route_rviz_;

  tf::TransformListener* listener_;

  int target_id_ = -1;
  bool target_id_updated_ = false;

  bool route_updated_ = false;

  bool smoothed_route_path_ = false;

  bool smoothed_success_ = false;

  std::vector<int> path_ids_;
  std::vector<TrajectoryPoint> smoothed_route_points_;
  std::vector<double> path_point_theta_;
  bool path_found_ = false;
  OsmMapManager* osm_map_manager_;
  osm_parser::path_finder_algorithm::Dijkstra dijkstra_planner_;

  int current_navigation_node_id_;

  int path_id_last_index_ = 0;

  geometry_msgs::PoseStamped local_navigation_pose_;

  //根据route的结果重新获取每个导航点的朝向theta值
  void GetThetaFromRoutePath();

  //对route得到的全局路径进行平滑
  bool SmootherRoutePath();

  double NormalAngle(double angle);

  void TargetPointCallback(const std_msgs::Int32::ConstPtr& msg);

  void MoveBaseGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  bool TransformPoint(tf::TransformListener* listener,
                      const std::string source_frame,
                      const std::string desire_frame,
                      geometry_msgs::PointStamped& source_pt,
                      geometry_msgs::PointStamped& desire_pt);

  bool TransformPose(tf::TransformListener* listener,
                     const std::string source_frame,
                     const std::string desire_frame,
                     geometry_msgs::PoseStamped& source_pose,
                     geometry_msgs::PoseStamped& desire_pose);
};
}  // namespace planning

#endif