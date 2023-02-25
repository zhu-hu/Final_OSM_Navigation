//
// Created by luyifan on 19-11-5.
//

#include "frame.h"

namespace planning {
Frame::Frame(const TrajectoryPoint &planning_start_point,
             const LocalView *local_view)
    : planning_start_point_(planning_start_point), local_view_(local_view) {}
Frame::Frame(const TrajectoryPoint &planning_start_point,
             Trajectory &last_frame_trajectory_, const LocalView *local_view)
    : planning_start_point_(planning_start_point),
      _last_frame_trajectory(last_frame_trajectory_),
      local_view_(local_view),
      _last_frame_trajectory_exist(true) {}

Frame::Frame(const TrajectoryPoint &planning_start_point,
             const TrajectoryPoint &planning_target_point,
             const LocalView *local_view, Parameter *in_param)
    : planning_start_point_(planning_start_point),
      planning_target_point_(planning_target_point),
      local_view_(local_view),
      param(in_param) {}

Frame::~Frame() {}

bool Frame::Init(std::vector<ReferenceLine> *reference_lines) {
  reference_lines_ = reference_lines;
  return true;
}
}  // namespace planning