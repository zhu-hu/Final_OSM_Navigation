/*
 * Copyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao Tong
 * University. All rights reserved.
 * Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.03.10
 * @ modified at 2020.08.13
 */

#ifndef COMMON_OBJECT_H_
#define COMMON_OBJECT_H_

#include <Eigen/Dense>
#include <opencv2/core/types.hpp>
#include <vector>

using namespace Eigen;

struct LidarObject {
  cv::RotatedRect rRect;
  std::vector<cv::Point> hull;
  float min_z;
  float max_z;
};

struct TrackingObject {
  VectorXf x;                      // state vector: [px py dx dy yaw vx vy]
  MatrixXf P;                      // state covariance matrix
  int id;                          // track id
  int sur_age = 0;                 // survival age
  int dis_age = 0;                 // disappear age
  double pre_timestamp;            // timestamp for last measurement
  cyber_msgs::Object meas_object;  // measurement object
};

#endif  // SENSOR_FUSION_TRACK_COMMON_H_
