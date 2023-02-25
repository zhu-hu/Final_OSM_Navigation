//
// Created by Huyao on 12/19/17.
//

#ifndef STATEMACHINE_PLANNING_H
#define STATEMACHINE_PLANNING_H

#include <iostream>

#include "manager/grid_map_manager.h"
// #include "manager/map_manager.h"
// #include "manager/obstacle_manager.h"
#include "manager/osm_map_manager.h"
#include "manager/osm_route_manager.h"
// #include "manager/reference_line_manager.h"
// #include "manager/route_manager.h"
// #include "manager/routing_manager.h"
#include "manager/self_state_manager.h"
// #include "manager/signal_manager.h"
#include "state/state_common.h"
// #include "trajectory/SpeedGenerator.h"
#include "parameter/Parameter.h"
// #include "common/tiggo/map_param.h"
// #include "utils/CommonFunc.h"
#include <unordered_map>

#include "common/params/vehicle_model.h"
#include "common/struct/scenario.h"
#include "manager/publisher_manager.h"
// #include "manager/unprotected_handler.h"
#include "planner/planner_factory.h"
#include "ros/ros.h"
#include "ros/time.h"

namespace planning {
inline double CutDistance(double raw_dis, double cut_dis) {
  return (raw_dis > cut_dis) ? (raw_dis - cut_dis) : 0;
}
class StPause;
class Stager
    : public sc::state_machine<Stager, StPause, boost::pool_allocator<char>,
                               sc::exception_translator<>> {
 public:
  Stager(Parameter *param);

  ~Stager();

  inline void SetLastFrameTrajectory(const Trajectory trajectory) {
    last_trajectory_ = trajectory;
    _last_frame_traj_exist = true;
  }

  inline Trajectory &GetLastFrameTrajectory() { return last_trajectory_; }

  std::vector<TrajectoryPoint> ComputeStitchingTrajectory(
      const cyber_msgs::LocalizationEstimate &adc_state,
      double planning_cycle_time);

  Planner *GetPlanner(const std::string type);

  TrajectoryPoint FindDerivedStartPoint(Trajectory last_trajectory_,
                                        TrajectoryPoint current,
                                        double threshold);

  inline void SetEmergencyMode(const uint32_t emergency_mode_) {
    _emergency_mode = emergency_mode_;
  }

  inline void CancelEmergency() { _emergency_mode = 0; }

  inline uint32_t GetEmergencyMode()  // 0不紧急,1紧急减速,2紧急刹停
  {
    return _emergency_mode;
  }

  // inline UnProtectedType SetUnprotectedType(
  //     const UnProtectedType unprotected_type) {
  //   _unprotected_type = unprotected_type;
  // }

  // inline UnProtectedType GetUnprotectedType() { return _unprotected_type; }

  inline void SetUnpretectedCheckTime(const double check_time) {
    _unprotected_check_time = check_time;
  }

  inline double GetUnpretectedCheckTime() { return _unprotected_check_time; }

 public:
  Parameter *params_;
  // RoutingManager *routing_manager_;
  // MapManager *map_manager_;
  SelfStateManager *self_state_;
  // ReferenceLineManager *reference_line_manager_;
  PublisherManager *publisher_manager_;
  // ObstacleManager *obstacle_manager_;
  // SignalManager *signal_manager_;
  GridMapManager *grid_map_manager_;
  // UnProtectedHandler *unprotected_handler_;
  // SpeedGenerator *speed_generator_;
  // RouteManager *route_manager_;
  OsmMapManager *osm_map_manager_;
  OsmRouteManager *osm_route_manager_;
  bool _last_frame_traj_exist = false;
  bool _need_stop_for_obstacles = false;
  double _end_time_stop_for_obstalce = 0;
  bool _forced_quit_task_flag = false;

 private:
  ros::NodeHandle nh_;
  std::unordered_map<std::string, Planner *> planners_;
  PlannerFactory planner_factory_;
  Trajectory last_trajectory_;
  uint32_t _emergency_mode = 0;  // 0不紧急,1紧急减速,2紧急刹停
  // UnProtectedType _unprotected_type;
  double _unprotected_check_time;
  /********     Declare Singleton    ********/
 public:
  static Stager *instance(Parameter *param) {
    static Stager instance(param);
    return &instance;
  }

 private:
  Stager(const Stager &);

  Stager &operator=(const Stager &);
  /******** End of Declare Singleton ********/
};

}  // namespace planning

#endif  // STATEMACHINE_PLANNING_H
