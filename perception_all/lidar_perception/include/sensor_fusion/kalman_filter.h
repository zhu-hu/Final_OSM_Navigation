/*
 * Cobjectyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao
 * Tong University. All rights reserved. Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.03.10
 */

#ifndef SENSOR_FUSION_KALMAN_FILTER_H_
#define SENSOR_FUSION_KALMAN_FILTER_H_

#include "common/util.h"

#include <Eigen/Dense>

#include "common/object.h"

using namespace Eigen;

class KalmanFilter {
 public:
  KalmanFilter();

  // initialize state vector and state covariance matrix when first measurement
  void Init(TrackingObject& object);

  // predicts the state and the state covariance matrix using the process model
  void Predict(double dt, TrackingObject& object);

  // updates the state and the state covariance matrix using measurement
  void Update(TrackingObject& object);

 private:
  // process noise vector
  VectorXf v_;

  // state transition matrix
  MatrixXf F_;

  // process covariance matrix
  MatrixXf Q_;

  // measurement matrix
  MatrixXf H_;

  // measurement covariance matrix
  MatrixXf R_;
};

#endif  // SENSOR_FUSION_KALMAN_FILTER_H_
