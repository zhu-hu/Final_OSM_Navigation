/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include <utility>
#include <vector>

namespace planning {

/*
 * @brief:
 * This class solve an optimization problem:
 * Y
 * |
 * |                       P(x1, y1)  P(x2, y2)
 * |            P(x0, y0)                       ... P(x(k-1), y(k-1))
 * |P(start)
 * |
 * |________________________________________________________ X
 *
 *
 * Given an initial set of points from 0 to k-1,  The goal is to find a set of
 * points which makes the line P(start), P0, P(1) ... P(k-1) "smooth".
 */

struct FemPosDeviationSmootherConfig {
  double weight_fem_pos_deviation;
  double weight_path_length;
  double weight_ref_deviation;
  int max_iter;
  double time_limit;
  double verbose;
  double scaled_termination;
  double warm_start;
};

class FemPosDeviationSmoother {
 public:
  explicit FemPosDeviationSmoother(const FemPosDeviationSmootherConfig& config);

  bool Solve(const std::vector<std::pair<double, double>>& raw_point2d,
             const std::vector<double>& bounds, std::vector<double>* opt_x,
             std::vector<double>* opt_y);

 private:
  FemPosDeviationSmootherConfig config_;
};

}  // namespace planning

//***** default settings in FemPosDeviationSmootherConfig  *****//
// double weight_fem_pos_deviation = 1.0e10;
// double weight_ref_deviation = 1.0;
// double weight_path_length = 1.0;
// // osqp settings
// int32 max_iter = 500;
// // time_limit set to be 0.0 meaning no time limit
// double time_limit = 0.0;
// bool verbose = false;
// bool scaled_termination = true;
// bool warm_start = true;

//***** Proto  Format *****//
// optional double weight_fem_pos_deviation = 2 [default = 1.0e10];
// optional double weight_ref_deviation = 3 [default = 1.0];
// optional double weight_path_length = 4 [default = 1.0];
// // osqp settings
// optional int32 max_iter = 100 [default = 500];
// // time_limit set to be 0.0 meaning no time limit
// optional double time_limit = 101 [default = 0.0];
// optional bool verbose = 102 [default = false];
// optional bool scaled_termination = 103 [default = true];
// optional bool warm_start = 104 [default = true];
