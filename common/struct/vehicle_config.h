/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 * @file vehicle_config.h
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief
 * @version IVFC20
 * @date 2020-09-30
 *
 * @maintainer
 */

#pragma once

namespace cyberc3 {
namespace common {
struct VehicleParam {
  double length;
  double width;
  double back_edge_to_center;
  double max_steer_angle;
  double steer_ratio;
  double wheel_base;
  VehicleParam(double length, double width, double back_edge_to_center,
               double max_steer_angle, double steer_ratio, double wheel_base)
      : length(length),
        width(width),
        back_edge_to_center(back_edge_to_center),
        max_steer_angle(max_steer_angle),
        steer_ratio(steer_ratio),
        wheel_base(wheel_base) {}
};
}  // namespace common
}  // namespace cyberc3
