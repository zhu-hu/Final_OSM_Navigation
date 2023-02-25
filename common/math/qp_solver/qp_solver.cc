/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 * @file qp_solver.cc
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief
 * @version IVFC20
 * @date 2020-09-22
 *
 * @maintainer
 */

#include "common/math/qp_solver/qp_solver.h"

namespace cyberc3 {
namespace common {
namespace math {

QpSolver::QpSolver(const Eigen::MatrixXd& kernel_matrix,
                   const Eigen::MatrixXd& offset,
                   const Eigen::MatrixXd& affine_inequality_matrix,
                   const Eigen::MatrixXd& affine_inequality_boundary,
                   const Eigen::MatrixXd& affine_equality_matrix,
                   const Eigen::MatrixXd& affine_equality_boundary)
    : kernel_matrix_(kernel_matrix),
      offset_(offset),
      affine_inequality_matrix_(affine_inequality_matrix),
      affine_inequality_boundary_(affine_inequality_boundary),
      affine_equality_matrix_(affine_equality_matrix),
      affine_equality_boundary_(affine_equality_boundary) {}

const Eigen::MatrixXd& QpSolver::params() const { return params_; }

const Eigen::MatrixXd& QpSolver::kernel_matrix() const {
  return kernel_matrix_;
}

const Eigen::MatrixXd& QpSolver::offset() const { return offset_; }

const Eigen::MatrixXd& QpSolver::affine_equality_matrix() const {
  return affine_equality_matrix_;
}

const Eigen::MatrixXd& QpSolver::affine_equality_boundary() const {
  return affine_equality_boundary_;
}

const Eigen::MatrixXd& QpSolver::affine_inequality_matrix() const {
  return affine_inequality_matrix_;
}

const Eigen::MatrixXd& QpSolver::affine_inequality_boundary() const {
  return affine_inequality_boundary_;
}

}  // namespace math
}  // namespace common
}  // namespace cyberc3
