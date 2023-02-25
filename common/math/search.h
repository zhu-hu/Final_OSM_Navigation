/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 * @file search.h
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief Search-related functions.
 * @version IVFC20
 * @date 2020-09-23
 *
 * @maintainer
 */

#pragma once

#include <functional>

/**
 * @namespace cyberc3::common::math
 * @brief cyberc3::common::math
 */
namespace cyberc3 {
namespace common {
namespace math {

/**
 * @brief Given a unimodal function defined on the interval,
 *        find a value on the interval to minimize the function.
 *        Reference: https://en.wikipedia.org/wiki/Golden-section_search
 * @param func The target single-variable function to minimize.
 * @param lower_bound The lower bound of the interval.
 * @param upper_bound The upper bound of the interval.
 * @param tol The tolerance of error.
 * @return The value that minimize the function fun.
 */
double GoldenSectionSearch(const std::function<double(double)> &func,
                           const double lower_bound, const double upper_bound,
                           const double tol = 1e-6);

}  // namespace math
}  // namespace common
}  // namespace cyberc3
