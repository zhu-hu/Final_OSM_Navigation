/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 * @file pose2d.h
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief
 * @version IVFC20
 * @date 2020-10-11
 *
 * @maintainer
 */

#pragma once

#include <cmath>
#include <string>

#include "common/math/vec2d.h"

/**
 * @namespace cyberc3::common::math
 * @brief cyberc3::common::math
 */
namespace cyberc3 {
namespace common {
namespace math {

class Pose2d {
 public:
  constexpr Pose2d(const double x, const double y, const double theta) noexcept
      : x_(x), y_(y), theta_(theta) {}

  constexpr Pose2d() noexcept : Pose2d(0, 0, 0) {}

  double x() const { return x_; }

  double y() const { return y_; }

  double theta() const { return theta_; }

  void set_x(const double x) { x_ = x; }

  void set_y(const double y) { y_ = y; }

  void set_theta(const double theta) { theta_ = theta; }

  double DistanceTo(const Pose2d &other) const;

  double DistanceTo(const Vec2d &other) const;

  double DistanceSquareTo(const Pose2d &other) const;

  double DistanceSquareTo(const Vec2d &other) const;

  Pose2d ToThisCoordinate(const Pose2d &other) const;

  Vec2d ToThisCoordinate(const Vec2d &other) const;

  Pose2d FromThisCoordinate(const Pose2d &other) const;

  Vec2d FromThisCoordinate(const Vec2d &other) const;

  bool operator==(const Pose2d &other) const;

  std::string DebugString() const;

 protected:
  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
};

}  // namespace math
}  // namespace common
}  // namespace cyberc3
