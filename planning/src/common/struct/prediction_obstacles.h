//
// Created by luyifan on 19-11-5.
//

#ifndef PLANNING_PREDICTION_OBSTACLES_H
#define PLANNING_PREDICTION_OBSTACLES_H

#include "string"
#include "vector"
#include "position.h"
#include "trajectory_point.h"
#include "map/hdmap/src/math/Polygon.h"

namespace planning
{
enum PerceptionObstacleState
{
  STATIC,
  DYNAMIC
};

enum PerceptionObstacleType
{
  UNKNOWN,
  PEDESTRIAN,
  CONE,
  CYCLIST,
  CAR,
  TRUCK,
  BUS
};

struct PredictionObstacle
{
  uint32_t id;
  uint32_t age;
  double timestamp;
  double velocity;
  PerceptionObstacleType type;
  PerceptionObstacleState motion_status;
  Position position;
  double theta;
  common::math::Polygon2d polygon2d;
  std::vector<std::vector<TrajectoryPoint>> future_trajectories;
};
} // namespace planning

#endif //PLANNING_PREDICTION_OBSTACLES_H
