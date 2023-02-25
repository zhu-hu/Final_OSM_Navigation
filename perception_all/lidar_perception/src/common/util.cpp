/*
 * Copyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao Tong
 * University. All rights reserved.
 * Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.02.20
 */

#include "common/util.h"

namespace common {

void TransformPointCloud(const PointTypeCloud& cloud_in,
                         const mrpt::poses::CPose3D& pose,
                         PointTypeCloud& cloud_out) {
  cloud_out.resize(cloud_in.size());
  for (size_t index = 0; index < cloud_in.size(); index++) {
    const auto& point_src = cloud_in[index];
    auto& point_dst = cloud_out[index];
    point_dst = point_src;
    pose.composePoint(point_src.x, point_src.y, point_src.z, point_dst.x,
                      point_dst.y, point_dst.z);
  }
}

void TransformPointCloud(const mrpt::poses::CPose3D& pose,
                         PointTypeCloud& cloud_out) {
  for (size_t index = 0; index < cloud_out.size(); index++) {
    auto& point_dst = cloud_out[index];
    pose.composePoint(point_dst.x, point_dst.y, point_dst.z, point_dst.x,
                      point_dst.y, point_dst.z);
  }
}

void CopyPointCloud(const PointTypeCloud& cloud_in, PointTypeCloud& cloud_out) {
  cloud_out.points.assign(cloud_in.points.begin(), cloud_in.points.end());
  cloud_out.resize(cloud_in.size());
}

void CopyPointCloud(const PointTypeCloud& cloud_in,
                    const std::vector<int>& indices,
                    PointTypeCloud& cloud_out) {
  cloud_out.resize(indices.size());
  for (size_t i = 0; i < indices.size(); i++) {
    cloud_out.points[i] = cloud_in.points[indices[i]];
  }
}

void InsertPointCloud(const PointTypeCloud& cloud_in,
                      PointTypeCloud& cloud_out) {
  size_t size = cloud_in.size() + cloud_out.size();
  cloud_out.points.insert(cloud_out.points.end(), cloud_in.points.begin(),
                          cloud_in.points.end());
  cloud_out.resize(size);
}

void GetMrptPose(const geometry_msgs::Pose& local_pose,
                 mrpt::poses::CPose3D& mrpt_pose) {
  double roll, pitch, yaw;
  tf::Matrix3x3(
      tf::Quaternion(local_pose.orientation.x, local_pose.orientation.y,
                     local_pose.orientation.z, local_pose.orientation.w))
      .getRPY(roll, pitch, yaw);
  mrpt_pose = mrpt::poses::CPose3D(local_pose.position.x, local_pose.position.y,
                                   local_pose.orientation.z, yaw, pitch, roll);
}

float GetYawWithPose(const geometry_msgs::Pose& pose) {
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(pose.orientation.x, pose.orientation.y,
                               pose.orientation.z, pose.orientation.w))
      .getRPY(roll, pitch, yaw);
  return static_cast<float>(yaw);
}

float ControlAngleRad(const float& angle) {
  float out_angle = angle;
  while (out_angle > M_PI) out_angle -= 2. * M_PI;
  while (out_angle < -M_PI) out_angle += 2. * M_PI;
  return out_angle;
}

void AlignAngleRad(const float& ref_angle, float& correct_angle,
                   bool& need_trans) {
  std::vector<float> correct_angles(4);
  correct_angles[0] = correct_angle;
  float min_dif_angle =
      fabs(ControlAngleRad(fabs(correct_angles[0] - ref_angle)));
  int min_index = 0;
  for (int index = 1; index < 4; index++) {
    correct_angles[index] =
        ControlAngleRad(correct_angles[index - 1] - 0.5 * M_PI);
    float dif_angle =
        fabs(ControlAngleRad(fabs(correct_angles[index] - ref_angle)));
    if (dif_angle < min_dif_angle) {
      min_index = index;
      min_dif_angle = dif_angle;
    }
  }
  correct_angle = correct_angles[min_index];
  if (min_index == 1 || min_index == 3) {
    need_trans = true;
  } else {
    need_trans = false;
  }
}

void GetDifVector(const cyber_msgs::Object& meas_object,
                  const TrackingObject& track_object, VectorXf& dif) {
  dif = VectorXf(5);
  // correct angle of measurement
  bool need_trans;
  float meas_angle = ControlAngleRad(GetYawWithPose(meas_object.pose));
  float meas_dx = meas_object.dimensions.x;
  float meas_dy = meas_object.dimensions.y;
  AlignAngleRad(track_object.x(4), meas_angle, need_trans);
  if (need_trans) {
    float tmp = meas_dx;
    meas_dx = meas_dy;
    meas_dy = tmp;
  }
  Eigen::MatrixXf meas_points, track_points, dif_points;
  GetCornerPoints(meas_object.pose.position.x, meas_object.pose.position.y,
                  meas_dx, meas_dy, meas_angle, meas_points);
  GetCornerPoints(track_object.x(0), track_object.x(1), track_object.x(2),
                  track_object.x(3), track_object.x(4), track_points);
  dif_points = meas_points - track_points;
  std::vector<float> dif_vec(5);
  for (int i = 0; i < 5; i++) {
    dif_vec[i] = dif_points(i, 0) * dif_points(i, 0) +
                 dif_points(i, 1) * dif_points(i, 1);
  }
  int min_index = std::distance(
      dif_vec.begin(), std::min_element(dif_vec.begin(), dif_vec.end()));
  dif(0) = dif_points(min_index, 0);
  dif(1) = dif_points(min_index, 1);
  dif(2) = meas_dx - track_object.x(2);
  dif(3) = meas_dy - track_object.x(3);
  dif(4) = ControlAngleRad(meas_angle - track_object.x(4));
}

void GetCornerPoints(const float& px, const float& py, const float& dx,
                     const float& dy, const float& angle,
                     Eigen::MatrixXf& points_out) {
  points_out = Eigen::MatrixXf(5, 2);
  points_out << 0, 0, 0.5 * dx, 0.5 * dy, 0.5 * dx, -0.5 * dy, -0.5 * dx,
      -0.5 * dy, -0.5 * dx, 0.5 * dy;
  Eigen::MatrixXf R(2, 2);
  R << cos(angle), -sin(angle), sin(angle), cos(angle);
  points_out = (R * points_out.transpose()).transpose();
  Eigen::MatrixXf T(5, 2);
  T.col(0).fill(px);
  T.col(1).fill(py);
  points_out += T;
}
}  // namespace common
