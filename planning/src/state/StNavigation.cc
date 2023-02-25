#include "StNavigation.h"

namespace planning {

StNavigation::StNavigation(my_context ctx)
    : my_base(ctx), StBase<StNavigation>(std::string("StNavigation")) {}

StNavigation::~StNavigation() {}

sc::result StNavigation::react(const EvSysTick &evt) {
  //获取当前自身状态
  Stager &stager = context<Stager>();
  const auto &self_state_manager = stager.self_state_;
  const auto &adc_state = self_state_manager->GetLocalizationEstimation();

  const auto &publisher = stager.publisher_manager_;

  //获取局部栅格图管理器
  const auto &local_map_manager = stager.grid_map_manager_;

  const auto &grid_map = local_map_manager->grid_map();

  const auto &dilated_grid_map = local_map_manager->dilate_grid_map();

  const auto &fs_grid_map = local_map_manager->fs_grid_map();

  auto param = stager.params_;

  auto &osm_route_manager = stager.osm_route_manager_;

  // osm_route_manager->OsmRoutePlanner(247, 248);
  osm_route_manager->OsmRoutePlanner(adc_state);
  osm_route_manager->PublishRoutePath();
  osm_route_manager->PubSmoothedRoutePath();

  int current_navgation_node_id = osm_route_manager->CurrentNavigationNodeId();

  const auto navigation_point = osm_route_manager->osm_map_manager()
                                    ->osm_map()
                                    ->osm_nodes()[current_navgation_node_id];
  const auto navigation_point_local = osm_route_manager->LocalNavigationPose();
  //读取当前状态
  LocalView local_view;
  local_view.localization_estimate = &adc_state;
  local_view.grid_map = &grid_map;
  local_view.dilated_grid_map = &dilated_grid_map;
  local_view.fs_grid_map = &fs_grid_map;

  TrajectoryPoint start_point;
  start_point.x = 0.0;
  start_point.y = 0.0;
  start_point.theta = 0.0;

  TrajectoryPoint goal_point;
  goal_point.x = navigation_point_local.pose.position.x;
  goal_point.y = navigation_point_local.pose.position.y;
  goal_point.theta = tf::getYaw(navigation_point_local.pose.orientation);

  AWARN << "goal x : " << goal_point.x << ", goal y : " << goal_point.y
        << ", goal theta : " << goal_point.theta;
  Frame frame(start_point, goal_point, &local_view, param);
  frame.set_planning_target_speed(8.0);  // kmh

  auto dubins_planner = stager.GetPlanner("dubins");

  if (dubins_planner == nullptr) {
    return forward_event();
  }
  Trajectory *const best_trajectory = dubins_planner->Plan(&frame);

  //发布给控制，没有轨迹也要发给控制端，让控制端进行制动
  publisher->PublishTrajectory(best_trajectory);
  if (best_trajectory == nullptr) {
    publisher->PublishText("Best trajectory is null!");
    publisher->PublishStage("StNavigation");
    return forward_event();
  } else {
    AINFO << "Best Trajectory is ok!";

    std::vector<Trajectory> traj_vec_for_vis;
    traj_vec_for_vis.emplace_back(*best_trajectory);
    publisher->PublishVisualizationTrajectories(traj_vec_for_vis);

    return forward_event();
  }
}

sc::result StNavigation::react(const sc::exception_thrown &evt) {
  return forward_event();
}

}  // namespace planning