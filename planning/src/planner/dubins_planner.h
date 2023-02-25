// Created by zhu-hu

#ifndef PLANNING_DUBINSPLANNER_H
#define PLANNING_DUBINSPLANNER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>

#include "dubins_curve.h"
#include "planner.h"

namespace planning {
class DubinsPlanner : public Planner {
 public:
  DubinsPlanner(std::string n) {
    name_ = n;
    AINFO << "[PLANNER] " << name() << " is registered!!";

    pub_trajs_ =
        pnh_.advertise<visualization_msgs::MarkerArray>("/rviz_trajs", 5);
  }

  Trajectory *const Plan(Frame *frame);

 private:
  ros::NodeHandle pnh_;

  ros::Publisher pub_trajs_;

  std::vector<Trajectory> trajectories_;
  cv::Mat grid_map_;

  cv::Mat dilated_grid_map_;

  cv::Mat fs_grid_map_;

  //判断轨迹是否发生碰撞
  bool IsTrajectoryCollison(const hybrid_a_star::DubinsCurve &curve);

  //从DubinsPath中生成Trajectory
  void GenerateTrajectory(const hybrid_a_star::DubinsCurve &curve, int index);

  //判断终点位置是否被栅格占据
  bool EndPointOccupied(const double *end);

  //来评价vector<Trajectory>中的各条轨迹，给他们进行优先级排序
  void JudgeTrajectories();

  void PubTrajs();
};
}  // namespace planning

#endif