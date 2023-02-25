/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 * @file pose2d.cc
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief
 * @version IVFC20
 * @date 2020-10-11
 *
 * @maintainer
 */

#include "common/math/pose2d.h"

#include <cmath>

#include "absl/strings/str_cat.h"
#include "common/log.h"

namespace cyberc3 {
namespace common {
namespace math {

double Pose2d::DistanceTo(const Pose2d &other) const {
  return std::hypot(x_ - other.x_, y_ - other.y_);
}

double Pose2d::DistanceTo(const Vec2d &other) const {
  return std::hypot(x_ - other.x(), y_ - other.y());
}

double Pose2d::DistanceSquareTo(const Pose2d &other) const {
  const double dx = x_ - other.x_;
  const double dy = y_ - other.y_;
  return dx * dx + dy * dy;
}

double Pose2d::DistanceSquareTo(const Vec2d &other) const {
  const double dx = x_ - other.x();
  const double dy = y_ - other.y();
  return dx * dx + dy * dy;
}

Pose2d Pose2d::ToThisCoordinate(const Pose2d &other) const {
  return Pose2d((other.x() - x_) * cos(theta_) + (other.y() - y_) * sin(theta_),
                (other.y() - y_) * cos(theta_) - (other.x() - x_) * sin(theta_),
                other.theta() - theta_);
}
Vec2d Pose2d::ToThisCoordinate(const Vec2d &other) const {
  return Vec2d((other.x() - x_) * cos(theta_) + (other.y() - y_) * sin(theta_),
               (other.y() - y_) * cos(theta_) - (other.x() - x_) * sin(theta_));
}

Pose2d Pose2d::FromThisCoordinate(const Pose2d &other) const {
  return Pose2d(x_ + other.x() * cos(theta_) - other.y() * sin(theta_),
                y_ + other.y() * cos(theta_) + other.x() * sin(theta_),
                other.theta() + theta_);
}

Vec2d Pose2d::FromThisCoordinate(const Vec2d &other) const {
  return Vec2d(x_ + other.x() * cos(theta_) - other.y() * sin(theta_),
               y_ + other.y() * cos(theta_) + other.x() * sin(theta_));
}

bool Pose2d::operator==(const Pose2d &other) const {
  return (std::abs(x_ - other.x()) < kMathEpsilon &&
          std::abs(y_ - other.y()) < kMathEpsilon &&
          std::abs(theta_ - other.theta()) < kMathEpsilon);
}

std::string Pose2d::DebugString() const {
  return absl::StrCat("pose2d ( x = ", x_, "  y = ", y_, "  theta = ", theta_,
                      " )");
}

}  // namespace math
}  // namespace common
}  // namespace cyberc3
