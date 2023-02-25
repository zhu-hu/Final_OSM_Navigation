/*
 * Copyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao Tong
 * University. All rights reserved.
 * Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.02.21
 */

#include "grid_map.h"

GridMap::GridMap(const GridMapParams &params) {
    params_ = params;
    // roi grid map
    roi_map_height_ =
      static_cast<int>((params_.roi_params.max_x - params_.roi_params.min_x)
                       * params_.roi_params.pixel_scale);
    roi_map_width_ =
      static_cast<int>((params_.roi_params.max_y - params_.roi_params.min_y)
                       * params_.roi_params.pixel_scale);
    roi_map_height_origin_ = static_cast<int>(params_.roi_params.max_x
                                              * params_.roi_params.pixel_scale);
    roi_map_width_origin_ = static_cast<int>(params_.roi_params.max_y
                                             * params_.roi_params.pixel_scale);
}

void GridMap::GenerateGridMap(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                              cv::Mat &max_height_map,
                              cv::Mat &min_height_map) {
    max_height_map = cv::Mat_<float>(roi_map_height_, roi_map_width_, -3.0);
    min_height_map = cv::Mat_<float>(roi_map_height_, roi_map_width_, 3.0);
    for (int i = 0; i < cloud_in.size(); ++i) {
        int col, row;
        const pcl::PointXYZI &pt = cloud_in.points[i];
        LocalToPixel(pt.x, pt.y, row, col);
        if (row >= 0 && row < roi_map_height_ && col >= 0
            && col < roi_map_width_) {
            float max_height = max_height_map.at<float>(row, col);
            if (max_height < pt.z) {
                max_height_map.at<float>(row, col) = pt.z;
            }
        }
    }
}

void GridMap::UpdateGridMap(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                            cv::Mat &max_height_map) {
    for (int i = 0; i < cloud_in.size(); ++i) {
        int col, row;
        const pcl::PointXYZI &pt = cloud_in.points[i];
        LocalToPixel(pt.x, pt.y, row, col);
        if (row >= 0 && row < roi_map_height_ && col >= 0
            && col < roi_map_width_) {
            float max_height = max_height_map.at<float>(row, col);
            if (max_height < pt.z) {
                max_height_map.at<float>(row, col) = pt.z;
            }
        }
    }
}

bool GridMap::isHeightObstacle(const pcl::PointXYZI &pt,
                               const cv::Mat &max_height_map,
                               const cv::Mat &min_height_map) {

    int col, row;
    LocalToPixel(pt.x, pt.y, row, col);
    if (row >= 0 && row < roi_map_height_ && col >= 0 && col < roi_map_width_) {
        const float &max_height = max_height_map.at<float>(row, col);
        if (max_height > params_.height_thres) {
            return true;
        }
    }
    return false;
}

void GridMap::LocalToPixel(const float &real_x,
                           const float &real_y,
                           int &pixel_row,
                           int &pixel_coln) {
    pixel_row = roi_map_height_origin_
                - static_cast<int>(real_x * params_.roi_params.pixel_scale);
    pixel_coln = roi_map_width_origin_
                 - static_cast<int>(real_y * params_.roi_params.pixel_scale);
}

void GridMap::PixelToLocal(const float &pixel_row,
                           const float &pixel_coln,
                           float &real_x,
                           float &real_y) {
    real_x =
      (roi_map_height_origin_ - pixel_row) / params_.roi_params.pixel_scale;
    real_y =
      (roi_map_width_origin_ - pixel_coln) / params_.roi_params.pixel_scale;
}
