/*
 * Copyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao Tong
 * University. All rights reserved.
 * Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.02.25
 */

#ifndef LIDAR_PREPROCESS_LIDAR_PREPROCESS_H_
#define LIDAR_PREPROCESS_LIDAR_PREPROCESS_H_

#include <cyber_msgs/LocalizationEstimate.h>
#include <mrpt/poses/CPose3D.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Eigen>
#include <queue>

#include "common/point_type.h"
#include "common/util.h"
#include "ground_segmentation/line_fitting.h"

class LidarPreprocess {
 public:
  struct LidarPreprocessParams {
    // roi filter params(base_link coordinate)
    double near_noise_x_min = -1.25;
    double near_noise_x_max = 2.30;
    double near_noise_y_min = -0.40;
    double near_noise_y_max = 0.40;
    double livox_roi_x_min = -10.0;
    double livox_roi_x_max = 30.0;
    double livox_roi_y_min = -10.0;
    double livox_roi_y_max = 10.0;
    double livox_roi_z_min = -0.50;
    double livox_roi_z_max = 1.50;
    // remove close noise
    bool livox_noise_flag = false;
    double livox_noise_x_min = -5.00;
    double livox_noise_x_max = 10.00;
    double livox_noise_y_min = -6.00;
    double livox_noise_y_max = 6.00;
    double livox_noise_rate = 0.0015;
    double min_direct_remove_intensity = 1.0;
    // remove trailer noise
    double trailer_x_min = -1.70;
    double trailer_y_min = -1.30;
    double trailer_y_max = 1.30;
    // remove outlier
    double min_outlier_intensity = 5.5;
    double search_radius = 1.0;
    int min_neighbors = 5;
    // point cloud merging
    int frame_merging_count = 6;
    // ground segmentation params
    LineFittingGroundSegmenter::LineFittingParams gs_params;
  };

  LidarPreprocess(const LidarPreprocessParams& params);

  void LivoxPreprocess(const PointTypeCloud& cloud_in,
                       const mrpt::poses::CPose3D& pose, bool& send,
                       PointTypeCloud& cloud_out);

  void LivoxPreprocess2(const PointTypeCloud& cloud_in,
                        PointTypeCloud& cloud_out);

 private:
  LidarPreprocessParams params_;

  // livox cloud
  PointTypeCloud livox_cloud_;
  std::queue<int> cloud_size_queue_;
  int cur_count_ = 0;
  int publish_count_ = 2;

  // ground segmentation
  LineFittingGroundSegmenter* ground_segmenter_;

 private:
  void CloudPreprocess(const PointTypeCloud& cloud_in,
                       PointTypeCloud& cloud_out);

  void OutlierRemover(const PointTypeCloud& cloud_in,
                      PointTypeCloud& cloud_out);
};

#endif  // LIDAR_PREPROCESS_LIDAR_PREPROCESS_H_
