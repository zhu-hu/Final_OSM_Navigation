/*
 * Copyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao Tong
 * University. All rights reserved.
 * Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.02.25
 */

#include "lidar_preprocess/lidar_preprocess.h"

LidarPreprocess::LidarPreprocess(const LidarPreprocessParams& params) {
  params_ = params;
  double roi_r_min = std::abs(params_.near_noise_x_min);
  if (roi_r_min > std::abs(params_.near_noise_x_max)) {
    roi_r_min = std::abs(params_.near_noise_x_max);
  }
  if (roi_r_min > std::abs(params_.near_noise_y_min)) {
    roi_r_min = std::abs(params_.near_noise_y_min);
  }
  if (roi_r_min > std::abs(params_.near_noise_y_max)) {
    roi_r_min = std::abs(params_.near_noise_y_max);
  }
  double roi_r_max = std::abs(params_.livox_roi_x_min);
  if (roi_r_max < std::abs(params_.livox_roi_x_max)) {
    roi_r_max = std::abs(params_.livox_roi_x_max);
  }
  if (roi_r_max < std::abs(params_.livox_roi_y_min)) {
    roi_r_max = std::abs(params_.livox_roi_y_min);
  }
  if (roi_r_max < std::abs(params_.livox_roi_y_max)) {
    roi_r_max = std::abs(params_.livox_roi_y_max);
  }
  params_.gs_params.roi_r_min = roi_r_min;
  params_.gs_params.roi_r_max = roi_r_max;
  ground_segmenter_ = new LineFittingGroundSegmenter(params_.gs_params);
}

void LidarPreprocess::LivoxPreprocess(const PointTypeCloud& cloud_in,
                                      const mrpt::poses::CPose3D& pose,
                                      bool& send, PointTypeCloud& cloud_out) {
  cur_count_++;
  send = false;
  PointTypeCloudPtr filter_cloud_ptr(new PointTypeCloud);
  CloudPreprocess(cloud_in, *filter_cloud_ptr);
  PointTypeCloudPtr obstacle_cloud_ptr(new PointTypeCloud);
  if (filter_cloud_ptr->size() > 0) {
    ground_segmenter_->Segmenter(*filter_cloud_ptr, *obstacle_cloud_ptr);
  }
  common::TransformPointCloud(pose, *obstacle_cloud_ptr);
  if (cur_count_ > params_.frame_merging_count) {
    int first_cloud_size = cloud_size_queue_.front();
    cloud_size_queue_.pop();
    livox_cloud_.erase(livox_cloud_.begin(),
                       livox_cloud_.begin() + first_cloud_size);
  }
  common::InsertPointCloud(*obstacle_cloud_ptr, livox_cloud_);
  cloud_size_queue_.push(obstacle_cloud_ptr->points.size());
  if (cur_count_ % publish_count_ == 0) {
    send = true;
    mrpt::poses::CPose3D pose_inv(pose);
    pose_inv.inverse();
    PointTypeCloudPtr local_cloud_ptr(new PointTypeCloud);
    common::TransformPointCloud(livox_cloud_, pose_inv, *local_cloud_ptr);
    OutlierRemover(*local_cloud_ptr, cloud_out);
  }
  if (cur_count_ == 100000 * params_.frame_merging_count)
    cur_count_ = params_.frame_merging_count;
}

void LidarPreprocess::LivoxPreprocess2(const PointTypeCloud& cloud_in,
                                       PointTypeCloud& cloud_out) {
  PointTypeCloudPtr filter_cloud_ptr(new PointTypeCloud);
  CloudPreprocess(cloud_in, *filter_cloud_ptr);
  cloud_out.clear();
  if (filter_cloud_ptr->size() > 0) {
    ground_segmenter_->Segmenter(*filter_cloud_ptr, cloud_out);
  }
}

void LidarPreprocess::CloudPreprocess(const PointTypeCloud& cloud_in,
                                      PointTypeCloud& cloud_out) {
  std::vector<int> indices;
  size_t point_size = cloud_in.size();
  for (size_t i = 0; i < point_size; i++) {
    const PointType& cur_pt = cloud_in.points[i];
    if (cur_pt.x > params_.livox_roi_x_min &&
        cur_pt.x < params_.livox_roi_x_max &&
        cur_pt.y > params_.livox_roi_y_min &&
        cur_pt.y < params_.livox_roi_y_max &&
        cur_pt.z > params_.livox_roi_z_min &&
        cur_pt.z < params_.livox_roi_z_max &&
        !(cur_pt.x > params_.near_noise_x_min &&
          cur_pt.x < params_.near_noise_x_max &&
          cur_pt.y > params_.near_noise_y_min &&
          cur_pt.y < params_.near_noise_y_max) &&
        !(cur_pt.x > params_.trailer_x_min &&
          cur_pt.x < params_.near_noise_x_min &&
          cur_pt.y > params_.trailer_y_min &&
          cur_pt.y < params_.trailer_y_max)) {
      if (params_.livox_noise_flag && cur_pt.x > params_.livox_noise_x_min &&
          cur_pt.x < params_.livox_noise_x_max &&
          cur_pt.y > params_.livox_noise_y_min &&
          cur_pt.y < params_.livox_noise_y_max) {
        if (cur_pt.intensity < params_.min_direct_remove_intensity) {
          continue;
        }
        const PointType& prev_pt = cloud_in.points[(i - 1) % point_size];
        const PointType& next_pt = cloud_in.points[(i + 1) % point_size];
        float prev_delta_x = prev_pt.x - cur_pt.x;
        float prev_delta_y = prev_pt.y - cur_pt.y;
        float prev_distance =
            prev_delta_x * prev_delta_x + prev_delta_y * prev_delta_y;
        float next_delta_x = next_pt.x - cur_pt.x;
        float next_delta_y = next_pt.y - cur_pt.y;
        float next_distance =
            next_delta_x * next_delta_x + next_delta_y * next_delta_y;
        float threshold = params_.livox_noise_rate *
                          (cur_pt.x * cur_pt.x + cur_pt.y * cur_pt.y);
        if (prev_distance < threshold && next_distance < threshold)
          indices.push_back(i);
      } else {
        indices.push_back(i);
      }
    }
  }

  std::vector<int> final_indices;
  if (params_.livox_noise_flag) {
    size_t remain_size = indices.size();
    for (size_t i = 0; i < remain_size; i++) {
      int cur_index = indices[i];
      const PointType& cur_pt = cloud_in.points[cur_index];
      if (cur_pt.x > params_.livox_noise_x_min &&
          cur_pt.x < params_.livox_noise_x_max &&
          cur_pt.y > params_.livox_noise_y_min &&
          cur_pt.y < params_.livox_noise_y_max) {
        int prev_index = indices[(i - 1) % remain_size];
        int next_index = indices[(i + 1) % remain_size];
        const PointType& prev_pt = cloud_in.points[prev_index];
        const PointType& next_pt = cloud_in.points[next_index];
        float prev_delta_x = prev_pt.x - cur_pt.x;
        float prev_delta_y = prev_pt.y - cur_pt.y;
        float prev_distance =
            prev_delta_x * prev_delta_x + prev_delta_y * prev_delta_y;
        float next_delta_x = next_pt.x - cur_pt.x;
        float next_delta_y = next_pt.y - cur_pt.y;
        float next_distance =
            next_delta_x * next_delta_x + next_delta_y * next_delta_y;
        float threshold = params_.livox_noise_rate *
                          (cur_pt.x * cur_pt.x + cur_pt.y * cur_pt.y);
        if (prev_distance < threshold && next_distance < threshold)
          final_indices.push_back(cur_index);
      } else {
        final_indices.push_back(cur_index);
      }
    }
  }

  PointTypeCloudPtr filter_cloud_ptr(new PointTypeCloud);
  if (params_.livox_noise_flag) {
    common::CopyPointCloud(cloud_in, final_indices, *filter_cloud_ptr);
  } else {
    common::CopyPointCloud(cloud_in, indices, *filter_cloud_ptr);
  }

  if (filter_cloud_ptr->size() > 0) {
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(filter_cloud_ptr);
    float voxel_size = 0.2;
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.filter(cloud_out);
  }
}

void LidarPreprocess::OutlierRemover(const PointTypeCloud& cloud_in,
                                     PointTypeCloud& cloud_out) {
  if (cloud_in.size() == 0) {
    cloud_out.clear();
    return;
  }
  std::vector<int> candidate_indices, remain_indices;
  for (size_t i = 0; i < cloud_in.size(); i++) {
    const PointType& pt = cloud_in.points[i];
    if (pt.intensity < params_.min_outlier_intensity) {
      candidate_indices.push_back(i);
    } else {
      remain_indices.push_back(i);
    }
  }
  PointTypeCloudPtr canditate_cloud_ptr(new PointTypeCloud);
  common::CopyPointCloud(cloud_in, candidate_indices, *canditate_cloud_ptr);
  common::CopyPointCloud(cloud_in, remain_indices, cloud_out);

  if (canditate_cloud_ptr->size() > 0) {
    PointTypeCloudPtr filter_cloud_ptr(new PointTypeCloud);
    pcl::RadiusOutlierRemoval<PointType> outrem;
    outrem.setInputCloud(canditate_cloud_ptr);
    outrem.setRadiusSearch(params_.search_radius);
    outrem.setMinNeighborsInRadius(params_.min_neighbors);
    outrem.filter(*filter_cloud_ptr);
    common::InsertPointCloud(*filter_cloud_ptr, cloud_out);
  }
}
