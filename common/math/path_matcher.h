/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 * @file path_matcher.h
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief
 * @version IVFC20
 * @date 2020-09-23
 *
 * @maintainer
 */

#pragma once

#include <utility>
#include <vector>

#include "common/struct/pnc_point.h"

namespace cyberc3 {
namespace common {
namespace math {

class PathMatcher {
 public:
  PathMatcher() = delete;

  static PathPoint MatchToPath(const std::vector<PathPoint>& reference_line,
                               const double x, const double y);

  static std::pair<double, double> GetPathFrenetCoordinate(
      const std::vector<PathPoint>& reference_line, const double x,
      const double y);

  static PathPoint MatchToPath(const std::vector<PathPoint>& reference_line,
                               const double s);

 private:
  static PathPoint FindProjectionPoint(const PathPoint& p0, const PathPoint& p1,
                                       const double x, const double y);
};

}  // namespace math
}  // namespace common
}  // namespace cyberc3
