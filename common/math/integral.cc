/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 * @file integral.cc
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief
 * @version IVFC20
 * @date 2020-09-23
 *
 * @maintainer
 */

#include "common/math/integral.h"

#include <array>

#include "common/log.h"

namespace cyberc3 {
namespace common {
namespace math {

double IntegrateBySimpson(const std::vector<double>& func, const double dx,
                          const std::size_t nsteps) {
  CHECK_EQ(1, nsteps & 1);
  double sum1 = 0.0;
  double sum2 = 0.0;
  for (std::size_t i = 1; i + 1 < nsteps; ++i) {
    if ((i & 1) != 0) {
      sum1 += func[i];
    } else {
      sum2 += func[i];
    }
  }
  return dx / 3.0 * (4.0 * sum1 + 2.0 * sum2 + func[0] + func[nsteps - 1]);
}

double IntegrateByTrapezoidal(const std::vector<double>& func, const double dx,
                              const std::size_t nsteps) {
  double sum = 0;
  for (std::size_t i = 1; i + 1 < nsteps; ++i) {
    sum += func[i];
  }
  return dx * sum + 0.5 * dx * (func[0] + func[nsteps - 1]);
}

}  // namespace math
}  // namespace common
}  // namespace cyberc3
