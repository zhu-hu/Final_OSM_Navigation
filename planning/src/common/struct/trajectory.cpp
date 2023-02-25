//
// Created by Chen Xiaofeng on 19-11-1.
//

#include "trajectory.h"

namespace planning {
Trajectory::Trajectory() {}

Trajectory::~Trajectory() {}

void Trajectory::UpdateDistance2Obstacle() {}

void Trajectory::UpdateDrivable() {}

void Trajectory::UpdateCost() {}

void Trajectory::UpdateStatus() {
  UpdateDistance2Obstacle();
  UpdateDrivable();
  UpdateCost();
}

bool Trajectory::CreateTrajectory(
    const std::vector<TrajectoryPoint> &trajectory, double expect_speed) {
  points_.clear();
  for (const auto &path_point : trajectory) {
    points_.push_back(path_point);
  }
  expect_speed_ = expect_speed;
  return true;
}
bool Trajectory::CreateTrajectory(
    const std::vector<TrajectoryPoint> &trajectory_points) {
  points_.clear();
  points_ = trajectory_points;
  return true;
}

bool Trajectory::CreateTrajectory(
    const std::vector<TrajectoryPoint> &trajectory_points, int index) {
  points_.clear();
  points_ = trajectory_points;
  sample_index_ = index;
  return true;
}

double Trajectory::GetMaxNudgeLength() {
  return std::max(
      std::max(longitude_distance_cone_, longitude_distance_pedestrian_),
      longitude_distance_static_obstacle_);
}

}  // namespace planning
