/*
 * Copyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao Tong
 * University. All rights reserved.
 * Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.02.20
 */

#ifndef COMMON_UTIL_H_
#define COMMON_UTIL_H_

#include <cyber_msgs/Object.h>
#include <geometry_msgs/Pose.h>
#include <mrpt/poses/CPose3D.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Dense>
#include <vector>

#include "common/object.h"
#include "common/point_type.h"

namespace common {
void TransformPointCloud(const PointTypeCloud& cloud_in,
                         const mrpt::poses::CPose3D& pose,
                         PointTypeCloud& cloud_out);

void TransformPointCloud(const mrpt::poses::CPose3D& pose,
                         PointTypeCloud& cloud_out);

void CopyPointCloud(const PointTypeCloud& cloud_in, PointTypeCloud& cloud_out);

void CopyPointCloud(const PointTypeCloud& cloud_in,
                    const std::vector<int>& indices, PointTypeCloud& cloud_out);

void InsertPointCloud(const PointTypeCloud& cloud_in,
                      PointTypeCloud& cloud_out);

void GetMrptPose(const geometry_msgs::Pose& local_pose,
                 mrpt::poses::CPose3D& mrpt_pose);

float GetYawWithPose(const geometry_msgs::Pose& pose);

float ControlAngleRad(const float& angle);

void AlignAngleRad(const float& ref_angle, float& correct_angle,
                   bool& need_trans);

void GetDifVector(const cyber_msgs::Object& meas_object,
                  const TrackingObject& track_object, VectorXf& dif);

void GetCornerPoints(const float& px, const float& py, const float& dx,
                     const float& dy, const float& angle,
                     Eigen::MatrixXf& points_out);
}  // namespace common

#endif  // COMMON_UTIL_H_
