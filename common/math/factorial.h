/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 * @file factorial.h
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief Meta programming for computing factorial
 * @version IVFC20
 * @date 2020-09-23
 *
 * @maintainer
 */

#pragma once

#include <cstdint>

namespace cyberc3 {
namespace common {
namespace math {

template <uint64_t N>
struct Factorial {
  enum { value = N * Factorial<N - 1>::value };
};

template <>
struct Factorial<0> {
  enum { value = 1 };
};

}  // namespace math
}  // namespace common
}  // namespace cyberc3
