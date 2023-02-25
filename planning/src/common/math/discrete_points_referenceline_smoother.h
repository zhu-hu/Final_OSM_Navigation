#ifndef DISCRETE_POINTS_REFERENCELINE_SMOOTHER_H
#define DISCRETE_POINTS_REFERENCELINE_SMOOTHER_H

#include <algorithm>
#include <iostream>
#include <vector>

#include "common/math/discrete_points_math.h"
#include "common/math/fem_pos_deviation_smoother.h"
#include "common/struct/trajectory_point.h"
#include "common/utils/log.h"

namespace planning {

class DiscretePointsReferencelineSmoother {
 public:
  DiscretePointsReferencelineSmoother(FemPosDeviationSmootherConfig& config)
      : config_(config) {}
  ~DiscretePointsReferencelineSmoother() {}

  bool Smoother(const std::vector<TrajectoryPoint>& raw_points,
                std::vector<TrajectoryPoint>& smoother_points,
                double lat_bound);

 private:
  bool FemPosSmooth(const std::vector<std::pair<double, double>>& raw_points,
                    const std::vector<double>& bounds,
                    std::vector<std::pair<double, double>>* ptr_smooth_points);

  void NormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  void DeNormalizePoints(std::vector<std::pair<double, double>>* xy_points);

  bool GeneratePointProfile(std::vector<TrajectoryPoint>& raw_points);
  double zero_x_;
  double zero_y_;
  FemPosDeviationSmootherConfig config_;
};
}  // namespace planning
#endif