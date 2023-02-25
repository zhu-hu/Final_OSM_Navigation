//
// Created by Huyao on 12/19/17.
//

#include "planning.h"

// #include <iostream>
// #include <ros/ros.h>
#include "log.h"
#include "state/StPause.h"
// #include "state/StRun.h"
// #include "state/StStop.h"

namespace planning {
Stager::Stager(Parameter *param) {
  params_ = param;
  // map_manager_ = new MapManager(param);
  self_state_ = new SelfStateManager(&nh_, param);
  // reference_line_manager_ = new ReferenceLineManager(&nh_, map_manager_,
  // param); routing_manager_ = new RoutingManager(&nh_, map_manager_, param);
  publisher_manager_ = new PublisherManager(&nh_);
  // obstacle_manager_ = new ObstacleManager(&nh_, map_manager_, params_);
  // signal_manager_ = new SignalManager(&nh_);
  grid_map_manager_ = new GridMapManager(&nh_, params_);
  // unprotected_handler_ = new UnProtectedHandler(&nh_, param);
  // speed_generator_ = new SpeedGenerator(param);
  // route_manager_ = new RouteManager(&nh_, param);
  osm_map_manager_ = new OsmMapManager(&nh_, param);
  osm_route_manager_ = new OsmRouteManager(&nh_, param, osm_map_manager_);

  //注册Planner
  // planners_["bezier"] = planner_factory_.GetPlanner(planner::BEZIER);
  // planners_["spline"] = planner_factory_.GetPlanner(planner::SPLINE);
  // planners_["lattice"] = planner_factory_.GetPlanner(planner::LATTICE);
  // planners_["reference_line"] =
  //     planner_factory_.GetPlanner(planner::REFERENCE_LINE);
  // planners_["dstar_rrt"] = planner_factory_.GetPlanner(planner::DSTAR_RRT);
  // planners_["frenet"] = planner_factory_.GetPlanner(planner::FRENET);
  // planners_["rrt"] = planner_factory_.GetPlanner(planner::RRT);
  // planners_["hybrid_a_star"] =
  //     planner_factory_.GetPlanner(planner::HYBRID_A_STAR);
  planners_["dubins"] = planner_factory_.GetPlanner(planner::DUBINS);

  google::InitGoogleLogging("TrajectoryGenerator");
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_log_dir = "~/testLog/";
  AINFO << "Log initiate success! ";
  //程序结束时，是否需要释放，否则造成野指针和内存泄漏
}

Stager::~Stager() {
  delete params_;
  delete grid_map_manager_;
  // delete routing_manager_;
  // delete map_manager_;
  delete self_state_;
  // delete route_manager_;
  delete osm_map_manager_;
  delete osm_route_manager_;

  google::ShutdownGoogleLogging();
}

std::vector<TrajectoryPoint> Stager::ComputeStitchingTrajectory(
    const cyber_msgs::LocalizationEstimate &adc_state,
    double planning_cycle_time) {
  std::vector<TrajectoryPoint> stitching_trajectory;
  TrajectoryPoint adc_state_next;
  adc_state_next.theta = tf::getYaw(adc_state.pose.orientation);
  double forward = adc_state.velocity.linear.x * planning_cycle_time;
  adc_state_next.x =
      adc_state.pose.position.x + forward * cos(adc_state_next.theta);
  adc_state_next.y =
      adc_state.pose.position.y + forward * sin(adc_state_next.theta);

  if (!last_trajectory_.points().empty()) {
    for (auto last_trajectory_point : last_trajectory_.points())
      stitching_trajectory.emplace_back(last_trajectory_point);
    TrajectoryPoint tmp = Stager::FindDerivedStartPoint(
        last_trajectory_, adc_state_next,
        0.3);  //当偏移30cm的时候，以自身的预测点为下个时刻的规划起始点
    stitching_trajectory.emplace_back(tmp);
    return stitching_trajectory;
  }

  TrajectoryPoint tmp;
  tmp.x = adc_state_next.x;
  tmp.y = adc_state_next.y;
  tmp.theta = adc_state_next.theta;
  stitching_trajectory.emplace_back(tmp);
  return stitching_trajectory;
}

Planner *Stager::GetPlanner(const std::string type) {
  auto iter = planners_.find(type);
  if (iter != planners_.end()) return iter->second;
  AERROR << "No planner called " << type << " in factory!!";
  return nullptr;
}

TrajectoryPoint Stager::FindDerivedStartPoint(Trajectory last_trajectory_,
                                              TrajectoryPoint current,
                                              double threshold) {
  if (last_trajectory_.points().size() == 0)
    return current;  //如果输入路径为空，则返回p自身
  TrajectoryPoint NearestPoint;
  for (auto tmp_p : last_trajectory_.points()) {
    double min_distance = std::numeric_limits<double>::max();
    double distance =
        sqrt(pow((current.x - tmp_p.x), 2) + pow((current.y - tmp_p.y), 2));
    if (distance < min_distance) {
      min_distance = distance;
      NearestPoint.x = tmp_p.x;
      NearestPoint.y = tmp_p.y;
      NearestPoint.theta = tmp_p.theta;
    }
  }
  double distance2current = sqrt(pow((current.x - NearestPoint.x), 2) +
                                 pow((current.y - NearestPoint.y), 2));
  if (distance2current > threshold) {
    // AINFO << "current position is far away from last_trajectory!";
    return current;
  } else
    return NearestPoint;
}
}  // namespace planning
