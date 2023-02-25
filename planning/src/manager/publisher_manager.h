#ifndef STATEMACHINE_PUBLISHERMANAGER_H
#define STATEMACHINE_PUBLISHERMANAGER_H

#include <iostream>
#include <string>

#include "common/struct/reference_line.h"
#include "cyber_msgs/LaneWaypoint.h"
#include "cyber_msgs/LocalTrajList.h"
#include "cyber_msgs/LocalTrajPoint.h"
#include "nav_msgs/Path.h"
#include "self_state_manager.h"
#include "std_msgs/Bool.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/Float64.h"

namespace planning {
enum StagerMode {
  STAGERNORMAL,
  STAGERSTRAIGHT,
  STAGERLEFTTURN,
  STAGERRIGHTTURN,
  STAGERUTURN,
  STAGERCOUNTRYROAD,
  STAGERROADCHECK,
  STAGERPAKING,
  STAGERSTART,
  STAGERLANECHANGE,
  STAGEREMERGENCYSTOP,
  STAGEREMERGENCYAVOID
};

class PublisherManager {
 public:
  PublisherManager(ros::NodeHandle *nh);
  ~PublisherManager();
  void PublishTrajectory(Trajectory *const best_trajectory);
  void PublishStage(const std::string stage);
  void PublishTrafficLight(
      int traffic_light_type_);  // 0为红灯，1为绿灯，2为未响应
  void PublishWaitTime(double time_, double pass_time_);
  void PublishPlanTime(double time_, double pass_time_, std::string able,
                       std::string planner);
  void PublishStagerMode(const StagerMode stager_mode,
                         const uint32_t &emergency_mode_);
  void PublishStagerDistance(double stager_distance);
  void PublishText(const std::string &text);
  void PublishText(const std::string &text, int color_type);
  void PublishPoint(const TrajectoryPoint &trajectorypoint);
  void ShowTrajectories(const std::vector<ReferenceLine> &lines);
  void PublishDisToStation(double distance);
  void PublishStationStatus(int station_status);
  void PublishParkingTrigger(bool parking_trigger);  // 1为启动自动泊车
  void PublishDriveMode(bool is_auto_drive);
  void PublishRoutingResult(
      const std::vector<cyber_msgs::LaneWaypoint> &routing_result_list);

  void PublishRRTPath(const nav_msgs::Path &path);
  void PublishLocalPoint(const TrajectoryPoint &point,
                         const std::string &color);

  //在rviz中显示所有的规划轨迹
  void PublishVisualizationTrajectories(
      const std::vector<Trajectory> &trajectories) const;

  void PublishLocalTarget(const TrajectoryPoint &point);

 private:
  ros::NodeHandle *nh_;
  ros::Publisher pub_trajectory_;
  ros::Publisher pub_state_;
  ros::Publisher pub_traffic_;
  ros::Publisher pub_stager_mode_;
  ros::Publisher pub_stager_distance_;
  ros::Publisher pub_text_;
  ros::Publisher pub_point_;
  ros::Publisher pub_trajectories_visualization_;
  ros::Publisher pub_dis_to_station_;
  ros::Publisher pub_station_status_;
  ros::Publisher pub_parking_trigger_;
  ros::Publisher pub_routing_;

  ros::Publisher pub_rrt_path_;
  ros::Publisher pub_local_point_;
  ros::Publisher pub_local_target_;
  ros::Publisher pub_visualization_trajectories_;
};
}  // namespace planning

#endif