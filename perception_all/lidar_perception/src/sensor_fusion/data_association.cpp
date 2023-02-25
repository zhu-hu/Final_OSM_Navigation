/*
 * Cobjectyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao
 * Tong University. All rights reserved. Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.03.10
 */

#include "sensor_fusion/data_association.h"

std::vector<std::pair<std::vector<int>, std::vector<int>>>
DataAssociation::GreedyAlgorithmLidar(
    const cyber_msgs::ObjectArray& meas_objects,
    const std::vector<TrackingObject>& track_objects) {
  std::vector<std::pair<std::vector<int>, std::vector<int>>> association_list;
  // no measurement objects
  if (meas_objects.objects.size() == 0) {
    association_list.resize(track_objects.size());
    for (size_t i = 0; i < track_objects.size(); i++) {
      std::vector<int> track_vec(1, i);
      std::vector<int> meas_vec(1, -1);
      association_list[i] = std::make_pair(track_vec, meas_vec);
    }
    return association_list;
  }
  // calculate association matrix between trackded list and measure list
  MatrixXf association_matrix(track_objects.size(),
                              meas_objects.objects.size());
  for (int i = 0; i < track_objects.size(); ++i) {
    for (int j = 0; j < meas_objects.objects.size(); ++j) {
      association_matrix(i, j) = AssociationValueTrackedLidar(
          meas_objects.objects[j], track_objects[i]);
    }
  }

  // put the min number index to association list until larger than threshold
  MatrixXf binary_association_matrix(track_objects.size(),
                                     meas_objects.objects.size());
  binary_association_matrix.fill(0);
  int r, c;
  if (association_matrix.size() > 0) {
    while (association_matrix.minCoeff(&r, &c) < lidar_threshold_) {
      const VectorXf& vec_row = binary_association_matrix.row(r);
      const VectorXf& vec_col = binary_association_matrix.col(c);
      int max_row_index, max_col_index;
      if (vec_row.maxCoeff(&max_col_index) > 0.0) {
        /* To do list
         * more than one measured object associated to the tracked object
         */
      } else if (vec_col.maxCoeff(&max_row_index) > 0.0) {
        /* To do list
         * more than one tracked object associated to the measured object
         */
      } else {
        binary_association_matrix(r, c) = 1.0;
      }
      association_matrix(r, c) = lidar_threshold_;
    }

    // check track_objects
    for (int i = 0; i < binary_association_matrix.rows(); ++i) {
      std::vector<int> track_vec(1, i);
      std::vector<int> meas_vec;
      for (int j = 0; j < binary_association_matrix.cols(); j++) {
        if (binary_association_matrix(i, j) > 0) {
          meas_vec.push_back(j);
        }
      }
      // association failure : old disappear
      if (meas_vec.size() == 0) meas_vec.push_back(-1);
      association_list.push_back(std::make_pair(track_vec, meas_vec));
    }

    // association failure : new measurement
    for (int i = 0; i < binary_association_matrix.cols(); ++i) {
      const VectorXf& vec_col = binary_association_matrix.col(i);
      int max_row_index;
      if (vec_col.maxCoeff(&max_row_index) == 0.0) {
        std::vector<int> track_vec(1, -1);
        std::vector<int> meas_vec(1, i);
        association_list.push_back(std::make_pair(track_vec, meas_vec));
      }
    }
  }
  return association_list;
}

float DataAssociation::AssociationValueTrackedLidar(
    const cyber_msgs::Object& meas_object, const TrackingObject& track_object) {
  VectorXf dif;
  common::GetDifVector(meas_object, track_object, dif);
  float scale_factor =
      sqrt((meas_object.dimensions.x + meas_object.dimensions.y) * 0.5 +
           (track_object.x(2) + track_object.x(3)) * 0.5);
  scale_factor = scale_factor > 0.0001 ? scale_factor : 0.0001;
  float pos_dif = (fabs(dif(0)) + fabs(dif(1))) / scale_factor;
  float size_dif = (fabs(dif(2)) + fabs(dif(3))) / scale_factor;
  return (pos_dif + size_weight_ * size_dif);
}
