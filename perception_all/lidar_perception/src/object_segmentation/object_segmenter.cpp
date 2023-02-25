/*
 * Copyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao Tong
 * University. All rights reserved.
 * Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.02.23
 */

#include "object_segmentation/object_segmenter.h"

ObjectSegmenter::ObjectSegmenter(const ObjectSegmenterParams& params) {
  params_ = params;
}

void ObjectSegmenter::Segmenter(const cv::Mat& grid_map,
                                const cv::Mat& max_height_map,
                                const cv::Mat& min_height_map,
                                std::vector<LidarObject>& small_objects,
                                std::vector<LidarObject>& big_objects) {
  cv::Mat grid_map_copy = grid_map.clone();
  // apply the specified morphology operations to cluster
  cv::Mat close_kernel = cv::getStructuringElement(
      cv::MORPH_RECT,
      cv::Size(2 * params_.close_kernel_x + 1, 2 * params_.close_kernel_y + 1),
      cv::Point(params_.close_kernel_x, params_.close_kernel_y));
  cv::morphologyEx(grid_map_copy, grid_map_copy, cv::MORPH_CLOSE, close_kernel);

  // find only external contours in grid map
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(grid_map_copy, contours, CV_RETR_EXTERNAL,
                   CV_CHAIN_APPROX_SIMPLE);

  if (contours.size() > 0) {
    // filter by size
    std::vector<int> small_objects_index;
    std::vector<int> big_objects_index;
    for (size_t i = 0; i < contours.size(); i++) {
      cv::RotatedRect min_rect = cv::minAreaRect(contours[i]);
      float width = min_rect.size.width;
      float height = min_rect.size.height;
      float area = min_rect.size.area();
      float max, min;
      if (width > height) {
        max = width;
        min = height;
      } else {
        min = width;
        max = height;
      }
      // small objects
      if (max <= params_.cone_params.size_max &&
          min_rect.center.x > params_.cone_params.cone_col_min &&
          min_rect.center.x < params_.cone_params.cone_col_max &&
          min_rect.center.y > params_.cone_params.cone_row_min &&
          min_rect.center.y < params_.cone_params.cone_row_max) {
        small_objects_index.push_back(i);
      } else if (max < params_.length_max && area < params_.area_max &&
                 min < params_.width_max &&
                 min > params_.cone_params.size_max) {
        big_objects_index.push_back(i);
      }
    }
    GetLidarObjects(grid_map, max_height_map, min_height_map, contours,
                    small_objects_index, small_objects);
    GetLidarObjects(grid_map, max_height_map, min_height_map, contours,
                    big_objects_index, big_objects);
  }
}

void ObjectSegmenter::GetLidarObjects(
    const cv::Mat& grid_map, const cv::Mat& max_height_map,
    const cv::Mat& min_height_map,
    const std::vector<std::vector<cv::Point>>& contours,
    const std::vector<int>& objects_index,
    std::vector<LidarObject>& out_objects) {
  out_objects.resize(objects_index.size());
  for (size_t i = 0; i < objects_index.size(); i++) {
    int ci = objects_index[i];
    const cv::Point* ppt[1] = {&(contours[ci][0])};
    int npt[] = {(int)contours[ci].size()};
    cv::Mat mask_ann = cv::Mat::zeros(grid_map.size(), CV_8UC1);
    cv::fillPoly(mask_ann, ppt, npt, 1, cv::Scalar(255));
    cv::Rect bRect = cv::boundingRect(contours[ci]);
    // get height and points
    float max_height = -3.0;
    float min_height = 3.0;
    std::vector<cv::Point> points;
    for (int col = bRect.x; col < bRect.x + bRect.width; col++) {
      for (int row = bRect.y; row < bRect.y + bRect.height; row++) {
        if (mask_ann.at<uchar>(row, col) > 0 &&
            grid_map.at<uchar>(row, col) > 0) {
          if (max_height < max_height_map.at<float>(row, col)) {
            max_height = max_height_map.at<float>(row, col);
          }
          if (min_height > min_height_map.at<float>(row, col)) {
            min_height = min_height_map.at<float>(row, col);
          }
          points.push_back(cv::Point(col, row));
        }
      }
    }
    LidarObject& object = out_objects[i];
    FitRotatedRect(points, object.rRect, object.hull);
    object.max_z = max_height;
    object.min_z = min_height;
  }
}

void ObjectSegmenter::FitRotatedRect(const std::vector<cv::Point>& points_in,
                                     cv::RotatedRect& rRect_out,
                                     std::vector<cv::Point>& polygon_out) {
  std::vector<cv::Point> hull;
  cv::convexHull(cv::Mat(points_in), hull);
  size_t hull_size = hull.size();
  cv::RotatedRect min_rRect = cv::minAreaRect(hull);
  if (hull_size > 2) {
    size_t point_size = points_in.size();
    Eigen::MatrixXf point_matrix(point_size, 2);
    for (size_t i = 0u; i < point_size; i++) {
      point_matrix.row(i) << points_in[i].x, points_in[i].y;
    }

    int max_count;
    float min_dist;
    float min_area;
    RectParams min_rect_params;

    for (size_t i = 0u; i < hull_size; i++) {
      const cv::Point& p1 = hull[i];
      const cv::Point& p2 = hull[(i + 1) % hull_size];
      RectParams rect_params;
      rect_params.p1 = p1;
      rect_params.p2 = p2;
      int count;
      float dist;
      CalculateDistance(point_matrix, count, dist, rect_params);
      float area = (rect_params.l1_c_max - rect_params.l1_c_min) *
                   (rect_params.l2_c_max - rect_params.l2_c_min);
      if (i == 0) {
        max_count = count;
        min_dist = dist;
        min_area = area;
        min_rect_params = rect_params;
      } else {
        if (count > max_count) {
          max_count = count;
          min_dist = dist;
          min_area = area;
          min_rect_params = rect_params;
        } else if (count == max_count) {
          if (dist < min_dist) {
            min_dist = dist;
            min_area = area;
            min_rect_params = rect_params;
          } else if (area < min_area) {
            min_area = area;
            min_rect_params = rect_params;
          }
        }
      }
    }

    min_rect_params.l1_length =
        min_rect_params.l1_c_max - min_rect_params.l1_c_min + 1;
    min_rect_params.l2_length =
        min_rect_params.l2_c_max - min_rect_params.l2_c_min + 1;
    min_rect_params.l1_angle =
        atan2f(min_rect_params.l1(1, 0), min_rect_params.l1(0, 0));
    if (min_rect_params.l1_angle < 0) min_rect_params.l1_angle += M_PI;
    min_rect_params.l2_angle =
        atan2f(min_rect_params.l2(1, 0), min_rect_params.l2(0, 0));
    if (min_rect_params.l2_angle < 0) min_rect_params.l2_angle += M_PI;
    float axis_angle, height, width;
    if (min_rect_params.l1_angle > min_rect_params.l2_angle) {
      axis_angle = min_rect_params.l2_angle;
      height = min_rect_params.l1_length;
      width = min_rect_params.l2_length;
    } else {
      axis_angle = min_rect_params.l1_angle;
      height = min_rect_params.l2_length;
      width = min_rect_params.l1_length;
    }
    Eigen::VectorXf pc(2, 1);
    Eigen::Matrix2f C, C_inv;
    C << min_rect_params.l1(0, 0), min_rect_params.l1(1, 0),
        min_rect_params.l2(0, 0), min_rect_params.l2(1, 0);
    C_inv = C.inverse();
    Eigen::MatrixXf D(2, 1);
    D << (min_rect_params.l1_c_max + min_rect_params.l1_c_min) * 0.5,
        (min_rect_params.l2_c_max + min_rect_params.l2_c_min) * 0.5;
    pc = C_inv * D;
    cv::RotatedRect fit_rRect(cv::Point(pc(0), pc(1)), cv::Size(width, height),
                              axis_angle / M_PI * 180);
    float fit_area = width * height;
    min_area = min_rRect.size.area();
    if (min_area > 0 && fit_area / min_area < 1.3) {
      rRect_out = fit_rRect;
    } else {
      rRect_out = min_rRect;
    }
  } else {
    rRect_out = min_rRect;
  }
  polygon_out = hull;
}

void ObjectSegmenter::CalculateDistance(const Eigen::MatrixXf& points_matrix,
                                        int& count, float& dist_sum,
                                        RectParams& param) {
  param.l1 << (param.p1.x - param.p2.x), (param.p1.y - param.p2.y);
  param.l1.normalize();
  if (param.l1(1, 0) < 0) param.l1 *= -1;
  param.l2 << -1.0 * (param.p1.y - param.p2.y), (param.p1.x - param.p2.x);
  param.l2.normalize();
  if (param.l2(1, 0) < 0) param.l2 *= -1;
  Eigen::VectorXf l1_c = points_matrix * param.l1;
  Eigen::VectorXf l2_c = points_matrix * param.l2;
  param.l1_c_max = l1_c.maxCoeff();
  param.l1_c_min = l1_c.minCoeff();
  param.l2_c_max = l2_c.maxCoeff();
  param.l2_c_min = l2_c.minCoeff();
  param.l1_max_dist =
      (param.l1_c_max - param.l1_c_min) * params_.max_dist_percent;
  param.l1_max_dist = (param.l1_max_dist > 1.0) ? param.l1_max_dist : 1.0;
  param.l1_max_dist = (param.l1_max_dist < 4.0) ? param.l1_max_dist : 4.0;
  param.l2_max_dist =
      (param.l2_c_max - param.l2_c_min) * params_.max_dist_percent;
  param.l2_max_dist = (param.l2_max_dist > 1.0) ? param.l2_max_dist : 1.0;
  param.l2_max_dist = (param.l2_max_dist < 4.0) ? param.l2_max_dist : 4.0;
  count = 0;
  dist_sum = 0.0;
  for (int i = 0; i < points_matrix.rows(); i++) {
    bool l1_flag = true;
    float dist = param.l1_c_max - l1_c(i);
    float dist_tmp = l1_c(i) - param.l1_c_min;
    if (dist_tmp < dist) dist = dist_tmp;
    dist_tmp = param.l2_c_max - l2_c(i);
    if (dist_tmp < dist) {
      dist = dist_tmp;
      l1_flag = false;
    }
    dist_tmp = l2_c(i) - param.l2_c_min;
    if (dist_tmp < dist) {
      dist = dist_tmp;
      l1_flag = false;
    }
    if (l1_flag) {
      if (dist <= param.l1_max_dist) count++;
    } else {
      if (dist <= param.l2_max_dist) count++;
    }
    dist_sum += dist;
  }
}