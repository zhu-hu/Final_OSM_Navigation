#include "discrete_points_referenceline_smoother.h"

namespace planning {

bool DiscretePointsReferencelineSmoother::Smoother(
    const std::vector<TrajectoryPoint>& raw_points,
    std::vector<TrajectoryPoint>& smoother_points, double lat_bound) {
  std::vector<double> lateralbound;
  for (size_t i = 0; i < raw_points.size(); i++) {
    lateralbound.emplace_back(lat_bound);
  }
  lateralbound.front() = 0.0;
  lateralbound.back() = 0.0;

  std::vector<std::pair<double, double>> xy_points;
  std::vector<std::pair<double, double>> smoothed;
  for (size_t i = 0; i < raw_points.size(); i++) {
    xy_points.emplace_back(raw_points[i].x, raw_points[i].y);
  }

  NormalizePoints(&xy_points);

  bool status = FemPosSmooth(xy_points, lateralbound, &smoothed);

  if (!status) {
    AERROR << "discrete_points referenceline smoother fails";
    return false;
  }

  DeNormalizePoints(&smoothed);

  smoother_points.clear();

  for (size_t i = 0; i < raw_points.size(); i++) {
    TrajectoryPoint tmp;
    tmp.x = smoothed[i].first;
    tmp.y = smoothed[i].second;
    tmp.l = 0.0;
    smoother_points.emplace_back(tmp);
  }

  if (GeneratePointProfile(smoother_points)) {
    return true;
  } else {
    return false;
  }
}

bool DiscretePointsReferencelineSmoother::FemPosSmooth(
    const std::vector<std::pair<double, double>>& raw_points,
    const std::vector<double>& bounds,
    std::vector<std::pair<double, double>>* ptr_smooth_points) {
  const auto& fem_config = config_;
  FemPosDeviationSmoother smoother(fem_config);
  std::vector<double> box_bounds = bounds;
  const double box_ratio = 1.0 / std::sqrt(2.0);

  for (auto& bound : box_bounds) {
    bound *= box_ratio;
  }

  std::vector<double> opt_x;
  std::vector<double> opt_y;
  bool status = smoother.Solve(raw_points, box_bounds, &opt_x, &opt_y);

  if (!status) {
    AERROR << "Fem Pos reference line smoothing failed";
    return false;
  }

  if (opt_x.size() < 2 || opt_y.size() < 2) {
    AERROR << "Return by fem pos smoother is wrong. Size smaller than 2 ";
    return false;
  }

  if (opt_x.size() != opt_y.size()) {
    AERROR << "x and y result size not equal";
    return false;
  }
  for (size_t i = 0; i < opt_x.size(); i++) {
    ptr_smooth_points->emplace_back(opt_x[i], opt_y[i]);
  }

  return true;
}

void DiscretePointsReferencelineSmoother::NormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  zero_x_ = xy_points->front().first;
  zero_y_ = xy_points->front().second;
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x - zero_x_,
                                               curr_y - zero_y_);
                  point = std::move(xy);
                });
}

void DiscretePointsReferencelineSmoother::DeNormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x + zero_x_,
                                               curr_y + zero_y_);
                  point = std::move(xy);
                });
}

bool DiscretePointsReferencelineSmoother::GeneratePointProfile(
    std::vector<TrajectoryPoint>& raw_points) {
  if (raw_points.size() == 0) return false;
  std::vector<std::pair<double, double>> xy_points;
  xy_points.clear();
  for (size_t i = 0; i < raw_points.size(); i++) {
    xy_points.emplace_back(std::make_pair(raw_points[i].x, raw_points[i].y));
  }
  std::vector<double> headings;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  std::vector<double> accumulated_s;
  if (DiscretePointsMath::ComputePathProfile(
          xy_points, &headings, &accumulated_s, &kappas, &dkappas) == false)
    return false;
  for (size_t i = 0; i < raw_points.size(); i++) {
    raw_points[i].theta = headings[i];
    raw_points[i].kappa = kappas[i];
    raw_points[i].s = accumulated_s[i];
  }
}

}  // namespace planning