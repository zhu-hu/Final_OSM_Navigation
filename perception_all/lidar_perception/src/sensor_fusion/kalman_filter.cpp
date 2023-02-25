/*
 * Cobjectyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao
 * Tong University. All rights reserved. Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.03.10
 */

#include "sensor_fusion/kalman_filter.h"

KalmanFilter::KalmanFilter() {
  // process noise is the covariance of acceleration noise for object,
  // the two dimensios are for ax and ay
  v_ = VectorXf(2);
  v_ << 0.2, 0.2;

  // state transition matrix will be calculate later, here is only for init
  F_ = MatrixXf::Identity(7, 7);

  // measurement matrix for lidar is a 5×7 matrix
  H_ = MatrixXf::Zero(5, 7);
  H_.topLeftCorner(5, 5) = MatrixXf::Identity(5, 5);

  // process covariance matrix will be calculate later, here is only for init
  Q_ = MatrixXf::Identity(7, 7);

  // measurement covariance matrix for lidar is a 5×5 matrix
  // while the diagonal elements is position measurement noise
  R_ = MatrixXf::Identity(5, 5);
  R_(0, 0) = R_(1, 1) = 0.05;
  R_(2, 2) = R_(3, 3) = 0.5;
  R_(4, 4) = 5;
}

void KalmanFilter::Init(TrackingObject& object) {
  // init state vector by all zero
  object.x = VectorXf(7);
  object.x.fill(0);

  // assign measurement to state vector
  object.x(0) = object.meas_object.pose.position.x;
  object.x(1) = object.meas_object.pose.position.y;
  object.x(2) = object.meas_object.dimensions.x;
  object.x(3) = object.meas_object.dimensions.y;
  object.x(4) =
      common::ControlAngleRad(common::GetYawWithPose(object.meas_object.pose));

  // init state covariance matrix by identity matrix
  object.P = MatrixXf::Identity(7, 7) * 10;
  // give high uncertainty to the unobservable initial velocities
  object.P(5, 5) = object.P(6, 6) = 500;
}

void KalmanFilter::Predict(double dt, TrackingObject& object) {
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  // update state transition matrix
  F_(0, 5) = dt;
  F_(1, 6) = dt;

  // update process covariance matrix
  Q_(0, 0) = dt_4 / 4 * v_(0);
  Q_(0, 5) = dt_3 / 2 * v_(0);
  Q_(1, 1) = dt_4 / 4 * v_(1);
  Q_(1, 6) = dt_3 / 2 * v_(1);
  Q_(5, 0) = dt_3 / 2 * v_(0);
  Q_(5, 5) = dt_2 * v_(0);
  Q_(6, 1) = dt_3 / 2 * v_(1);
  Q_(6, 6) = dt_2 * v_(1);

  // predict function
  object.x = F_ * object.x;
  object.x(4) = common::ControlAngleRad(object.x(4));
  MatrixXf Ft = F_.transpose();
  object.P = F_ * object.P * Ft + Q_;
}

void KalmanFilter::Update(TrackingObject& object) {
  // update function
  VectorXf y;
  common::GetDifVector(object.meas_object, object, y);
  MatrixXf Ht = H_.transpose();
  MatrixXf S = H_ * object.P * Ht + R_;
  MatrixXf Si = S.inverse();
  MatrixXf K = object.P * Ht * Si;

  // new estimate
  object.x = object.x + K * y;
  object.x(4) = common::ControlAngleRad(object.x(4));
  MatrixXf I = MatrixXf::Identity(object.x.size(), object.x.size());
  object.P = (I - K * H_) * object.P;
}
