/*
 * Copyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao Tong
 * University. All rights reserved.
 * Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.02.21
 */

#ifndef GRID_MAP_GRID_MAP_H
#define GRID_MAP_GRID_MAP_H

#include <geometry_msgs/Pose.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <iostream>
#include <list>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class GridMap {
public:
    struct RoiMapParams {
        double min_x = -10.0;
        double max_x = 30.0;
        double min_y = -10.0;
        double max_y = 10.0;
        int pixel_scale = 10;
    };  // struct RoiMapParams

    struct GridMapParams {
        RoiMapParams roi_params;
        float height_thres = 0.30;  //超过这个高度的不再判断是否为障碍物点
    };                              // GridMapParams

    GridMapParams params_;

    // roi grid map
    int roi_map_height_;
    int roi_map_width_;

    GridMap(const GridMapParams &params);

    void GenerateGridMap(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                         cv::Mat &max_height_map,
                         cv::Mat &min_height_map);

    void UpdateGridMap(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                       cv::Mat &max_height_map);

    bool isHeightObstacle(const pcl::PointXYZI &pt,
                          const cv::Mat &max_height_map,
                          const cv::Mat &min_height_map);

private:
    // roi grid map
    int roi_map_height_origin_;
    int roi_map_width_origin_;

private:
    // coordination transform: real to pixel
    void LocalToPixel(const float &real_x,
                      const float &real_y,
                      int &pixel_row,
                      int &pixel_column);

    // coordination transform: pixel to real
    void PixelToLocal(const float &pixel_row,
                      const float &pixel_column,
                      float &real_x,
                      float &real_y);
};

#endif  // GRID_MAP_GRID_MAP_H
