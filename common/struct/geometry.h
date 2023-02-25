/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 * @file geometry.h
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief
 * @version IVFC20
 * @date 2020-09-23
 *
 * @maintainer
 */

#pragma once

namespace cyberc3 {
namespace common {
struct Quaternion {
  double x;
  double y;
  double z;
  double w;
  Quaternion() : x(0.0), y(0.0), z(0.0), w(0.0) {}
};
}  // namespace common
}  // namespace cyberc3
