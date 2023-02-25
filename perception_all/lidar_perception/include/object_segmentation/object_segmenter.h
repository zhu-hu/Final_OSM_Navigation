/*
 * Copyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao Tong
 * University. All rights reserved.
 * Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.02.23
 */

#ifndef OBJECT_SEGMENTER_H_
#define OBJECT_SEGMENTER_H_

#include "common/util.h"

#include <cv_bridge/cv_bridge.h>
#include <cyber_msgs/ObjectArray.h>

#include <algorithm>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#include "common/object.h"
#include "grid_map/grid_map.h"

class ObjectSegmenter {
 public:
  struct ObjectSegmenterParams {
    float length_max = 120;  // 12m
    float area_max = 4800;   // 48 m^2
    float width_max = 30;    // 3.0m
    GridMap::ConeParams cone_params;
    int close_kernel_x = 1;
    int close_kernel_y = 4;
    float max_dist_percent = 0.15;
  };

  ObjectSegmenter(){};

  ObjectSegmenter(const ObjectSegmenterParams& params);

  void Segmenter(const cv::Mat& grid_map, const cv::Mat& max_height_map,
                 const cv::Mat& min_height_map,
                 std::vector<LidarObject>& small_objects,
                 std::vector<LidarObject>& big_objects);

 private:
  struct RectParams {
    cv::Point p1;
    cv::Point p2;
    Eigen::Vector2f l1;
    Eigen::Vector2f l2;
    float l1_c_max;
    float l1_c_min;
    float l2_c_max;
    float l2_c_min;
    float l1_max_dist;
    float l2_max_dist;
    float l1_length;
    float l2_length;
    float l1_angle;
    float l2_angle;
  };

  ObjectSegmenterParams params_;

  void GetLidarObjects(const cv::Mat& grid_map, const cv::Mat& max_height_map,
                       const cv::Mat& min_height_map,
                       const std::vector<std::vector<cv::Point>>& contours,
                       const std::vector<int>& objects_index,
                       std::vector<LidarObject>& out_objects);

  void FitRotatedRect(const std::vector<cv::Point>& points_in,
                      cv::RotatedRect& rRect_out,
                      std::vector<cv::Point>& polygon_out);

  void CalculateDistance(const Eigen::MatrixXf& points_matrix, int& count,
                         float& dist_sum, RectParams& param);
};

#endif  // OBJECT_SEGMENTER_H_
