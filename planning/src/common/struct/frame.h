//
// Created by luyifan on 19-11-5.
//

#ifndef PLANNING_FRAME_H
#define PLANNING_FRAME_H

#include <nav_msgs/Path.h>

#include <list>

#include "common/struct/reference_line.h"
#include "common/struct/trajectory_point.h"
#include "cyber_msgs/LocalizationEstimate.h"
#include "local_view.h"
#include "parameter/Parameter.h"

namespace planning {
struct DstarRRTTask {
  bool create;
  bool cancel;
  TrajectoryPoint goal;
};

struct RRTTask {
  //左下角为原点时,目标点的x,y坐标
  double x;
  double y;
  //目标点与当前位置的夹角,顺时针为负,逆时针为正
  double yaw;
};

enum MotionPlanningType { VELOCITYKEEPING = 0 };

class Frame {
 public:
  Frame(const TrajectoryPoint &planning_start_point,
        const LocalView *local_view);
  Frame(const TrajectoryPoint &planning_start_point,
        Trajectory &last_frame_trajectory_, const LocalView *local_view);
  Frame(const TrajectoryPoint &planning_start_point,
        const TrajectoryPoint &planning_target_point,
        const LocalView *local_view, Parameter *in_param);
  ~Frame();

  bool Init(std::vector<ReferenceLine> *reference_lines);

  inline const LocalView *local_view() { return local_view_; }

  // inline Parameter *param_() { return param_; }

  inline void set_planning_target_speed(double target_speed) {
    planning_target_speed_ = target_speed;
  }

  inline const double planning_target_speed() { return planning_target_speed_; }

  inline std::vector<ReferenceLine> *mutable_reference_lines() {
    return reference_lines_;
  }

  inline const TrajectoryPoint &planning_start_point() {
    return planning_start_point_;
  }

  inline const TrajectoryPoint &planning_target_point() {
    return planning_target_point_;
  }

  inline void set_task(DstarRRTTask *task) { task_ = task; }

  const inline DstarRRTTask *task() { return task_; }

  inline void set_rrt_task(RRTTask *rrt_task) { rrt_task_ = rrt_task; }

  const inline RRTTask *rrt_task() { return rrt_task_; }

  inline void set_rrt_path(nav_msgs::Path *path) { rrt_path_ = path; }

  const inline nav_msgs::Path *rrt_path() { return rrt_path_; }

  inline Trajectory &GetLastFrameTrajectory() { return _last_frame_trajectory; }

  inline void SetLastFrameTrajectory(Trajectory &traj_) {
    _last_frame_trajectory = traj_;
    _last_frame_trajectory_exist = true;
  }

  inline Trajectory &GetTrajectory() { return trajectory_; }
  inline void ChooseMotionPlanningType(MotionPlanningType planning_type_) {
    _planning_type = planning_type_;
  }

  inline const MotionPlanningType &GetMotionPlanningType() {
    return _planning_type;
  }

  inline bool JudgeLastFrameTrajectoryExist() {
    return _last_frame_trajectory_exist;
  }

  std::string current_unprotected_name_;

  double heading;

  Parameter *param;

  //规划出来的最终轨迹通过这个来接收并返回
  Trajectory derived_trajectory_;

 private:
  const TrajectoryPoint planning_start_point_;
  TrajectoryPoint planning_target_point_;
  Trajectory _last_frame_trajectory;
  Trajectory trajectory_;
  bool _last_frame_trajectory_exist = false;
  const LocalView *local_view_;
  MotionPlanningType _planning_type;
  std::vector<ReferenceLine> *reference_lines_;
  DstarRRTTask *task_;
  RRTTask *rrt_task_;
  nav_msgs::Path *rrt_path_;
  // Parameter *param_;
  double planning_target_speed_;
};
}  // namespace planning

#endif  // PLANNING_FRAME_H
