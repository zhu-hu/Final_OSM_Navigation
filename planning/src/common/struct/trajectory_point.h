//
// Created by luyifan on 19-11-5.
//

#ifndef PLANNING_TRAJECTORY_POINT_H
#define PLANNING_TRAJECTORY_POINT_H

#include <cmath>

namespace planning
{
struct TrajectoryPoint
{
  double x;
  double y;
  double theta;
  double s;
  double l;
  double t;
  double kappa;
  double velocity; //期望速度

  double l_dot;
  double l_dot_dot;
  double l_dot_dot_dot;
  double s_dot;
  double s_dot_dot;
  double s_dot_dot_dot;

  double c; //曲率
  double diff; //与前点间的距离
};
} // namespace planning

#endif //PLANNING_TRAJECTORY_POINT_H
