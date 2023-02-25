/*
 * Copyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao Tong
 * University. All rights reserved.
 * Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.02.21
 */

#ifndef GRID_MAP_GRID_MAP_H
#define GRID_MAP_GRID_MAP_H

#include <cyber_msgs/LocalizationEstimate.h>
#include <cyber_msgs/ObjectArray.h>
#include <geometry_msgs/Pose.h>
#include <mrpt/poses/CPose3D.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <iostream>
#include <list>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "common/object.h"
#include "common/point_type.h"
#include "common/util.h"

class GridMap {
 public:
  struct RoiMapParams {
    double min_x = -10.0;
    double max_x = 30.0;
    double min_y = -10.0;
    double max_y = 10.0;
    int pixel_scale = 10;
  };  // struct RoiMapParams

  struct PersonParams {
    float size_max = 14;     // 1.4m
    float size_min = 3;      // 0.3m
    float area_max = 100;    // 1 m^2
    float height_min = 0.6;  // 0.6m
  };

  struct ConeParams {
    float size_max = 2;        // 0.2m
    float research_dis = 2.0;  // 2.0m
    float height_max = 0.5;    // 0.5m
    float sur_time = 10;       // 10s
    float cone_x_min = 0.0;
    float cone_x_max = 30.0;
    float cone_y_min = -5.0;
    float cone_y_max = 5.0;
    int cone_row_min;
    int cone_row_max;
    int cone_col_min;
    int cone_col_max;
  };

  struct TrackingParams {
    float v_threshold = 2.0;
    int sur_age = 5;
  };

  struct EmergencyParams {
    float slow_down_x_ = 2.0;
    float slow_down_y_ = 1.0;
    float stop_x_ = 1.0;
    float stop_y_ = 0.5;
    float car_x_ = 2.4;
    float car_y_ = 0.5;
    int slow_down_row_min;
    int slow_down_row_max;
    int slow_down_col_min;
    int slow_down_col_max;
    int stop_row_min;
    int stop_row_max;
    int stop_col_min;
    int stop_col_max;
  };

  struct GridMapParams {
    RoiMapParams roi_params;
    PersonParams person_params;
    ConeParams cone_params;
    TrackingParams tracking_params;
    EmergencyParams safe_params;
  };  // GridMapParams

  GridMapParams params_;

  // roi grid map
  int roi_map_height_;
  int roi_map_width_;

  GridMap(const GridMapParams& params);

  void GenerateGridMap(const PointTypeCloud& cloud_in, cv::Mat& roi_grid_map,
                       cv::Mat& max_height_map, cv::Mat& min_height_map);

  void GenerateGridMap(const PointTypeCloud& cloud_in, cv::Mat& roi_grid_map,
                       int& safe);

  void GetGlobalBigObjects(const mrpt::poses::CPose3D& pose,
                           const std::vector<LidarObject>& lidar_objects,
                           cyber_msgs::ObjectArray& global_objects);

  void PostProcessBigObjects(const cyber_msgs::ObjectArray& in_objects,
                             cv::Mat& roi_grid_map,
                             cyber_msgs::ObjectArray& out_objects);

  void GetGlobalSmallObjects(const mrpt::poses::CPose3D& pose,
                             const std::vector<LidarObject>& lidar_objects,
                             const cyber_msgs::ObjectArray& big_objects,
                             cyber_msgs::ObjectArray& global_objects);

 private:
  // roi grid map
  int roi_map_height_origin_;
  int roi_map_width_origin_;

  // cone objects
  std::vector<std::pair<double, cyber_msgs::Object>> global_cone_objects_;

 private:
  void TransPolygon(const LidarObject& object_in,
                    const mrpt::poses::CPose3D& pose,
                    geometry_msgs::Polygon& polygon_out);

  // coordination transform: real to pixel
  void LocalToPixel(const float& real_x, const float& real_y, int& pixel_row,
                    int& pixel_column);

  // coordination transform: pixel to real
  void PixelToLocal(const float& pixel_row, const float& pixel_column,
                    float& real_x, float& real_y);

  void LocalToGlobal(const mrpt::poses::CPose3D& pose, const float& local_x,
                     const float& local_y, float& global_x, float& global_y);
};

#endif  // GRID_MAP_GRID_MAP_H
