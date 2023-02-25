/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 * @file search.cc
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief
 * @version IVFC20
 * @date 2020-09-23
 *
 * @maintainer
 */

#include "common/math/search.h"

#include <cmath>

namespace cyberc3 {
namespace common {
namespace math {

double GoldenSectionSearch(const std::function<double(double)> &func,
                           const double lower_bound, const double upper_bound,
                           const double tol) {
  static constexpr double gr = 1.618033989;  // (sqrt(5) + 1) / 2

  double a = lower_bound;
  double b = upper_bound;

  double t = (b - a) / gr;
  double c = b - t;
  double d = a + t;

  while (std::abs(c - d) > tol) {
    if (func(c) < func(d)) {
      b = d;
    } else {
      a = c;
    }
    t = (b - a) / gr;
    c = b - t;
    d = a + t;
  }
  return (a + b) * 0.5;
}

}  // namespace math
}  // namespace common
}  // namespace cyberc3
