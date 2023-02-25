/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved
 *
 * @file matrix_operations.cc
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief Defines some useful matrix operations.
 * @version IVFC20
 * @date 2020-09-23
 *
 * @maintainer
 */

#include "common/math/matrix_operations.h"

#include <cmath>
#include <utility>

#include "Eigen/Dense"
#include "Eigen/SVD"

/**
 * @namespace cyberc3::common::math
 * @brief The math namespace deals with a number of useful mathematical objects.
 */
namespace cyberc3 {
namespace common {
namespace math {

bool ContinuousToDiscrete(const Eigen::MatrixXd &m_a,
                          const Eigen::MatrixXd &m_b,
                          const Eigen::MatrixXd &m_c,
                          const Eigen::MatrixXd &m_d, const double ts,
                          Eigen::MatrixXd *ptr_a_d, Eigen::MatrixXd *ptr_b_d,
                          Eigen::MatrixXd *ptr_c_d, Eigen::MatrixXd *ptr_d_d) {
  if (ts <= 0.0) {
    AERROR << "ContinuousToDiscrete : ts is less than or equal to zero";
    return false;
  }

  // Only matrix_a is mandatory to be non-zeros in matrix
  // conversion.
  if (m_a.rows() == 0) {
    AERROR << "ContinuousToDiscrete: matrix_a size 0 ";
    return false;
  }

  if (m_a.cols() != m_b.rows() || m_b.cols() != m_d.cols() ||
      m_c.rows() != m_d.rows() || m_a.cols() != m_c.cols()) {
    AERROR << "ContinuousToDiscrete: matrix dimensions mismatch";
    return false;
  }

  Eigen::MatrixXd m_identity =
      Eigen::MatrixXd::Identity(m_a.cols(), m_a.rows());

  *ptr_a_d =
      (m_identity - ts * 0.5 * m_a).inverse() * (m_identity + ts * 0.5 * m_a);

  *ptr_b_d = std::sqrt(ts) * (m_identity - ts * 0.5 * m_a).inverse() * m_b;

  *ptr_c_d = std::sqrt(ts) * m_c * (m_identity - ts * 0.5 * m_a).inverse();

  *ptr_d_d = 0.5 * m_c * (m_identity - ts * 0.5 * m_a).inverse() * m_b + m_d;

  return true;
}

}  // namespace math
}  // namespace common
}  // namespace cyberc3
