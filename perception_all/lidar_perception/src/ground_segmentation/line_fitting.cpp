/*
 * Copyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao Tong
 * University. All rights reserved.
 * Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.02.21
 */

#include "ground_segmentation/line_fitting.h"

Bin::Bin() : min_z_(std::numeric_limits<double>::max()), has_point_(false) {}

Bin::Bin(const Bin& segment) {
  has_point_ = segment.has_point_;
  min_z_ = segment.min_z_;
  min_z_range_ = segment.min_z_range_;
  min_z_index_ = segment.min_z_index_;
}

void Bin::AddPoint(const PointType& point, const std::pair<int, int>& index) {
  const double d = sqrt(point.x * point.x + point.y * point.y);
  AddPoint(d, point.z, index);
}

void Bin::AddPoint(const double& d, const double& z,
                   const std::pair<int, int>& index) {
  has_point_ = true;
  if (z < min_z_) {
    min_z_ = z;
    min_z_range_ = d;
    min_z_index_ = index;
  }
}

Bin::MinZPoint Bin::GetMinZPoint() {
  MinZPoint point;

  if (has_point_) {
    point.z = min_z_;
    point.d = min_z_range_;
    point.index = min_z_index_;
  }

  return point;
}

Segment::Segment(LineFittingGroundSegmenter* ground_segmeter_ptr)
    : ground_segmeter_ptr_(ground_segmeter_ptr),
      bins_(ground_segmeter_ptr->params_.n_bins),
      start_bin_index_(std::numeric_limits<int>::max()),
      end_bin_index_(std::numeric_limits<int>::min()) {}

void Segment::FitSegmentLines() {
  if (start_bin_index_ > end_bin_index_) return;
  lines_.clear();
  double cur_height = ground_segmeter_ptr_->params_.prior_ground_z;
  std::list<Bin::MinZPoint> cur_line_points;
  auto line_start = bins_.begin() + start_bin_index_;
  auto line_end = bins_.begin() + end_bin_index_ + 1;
  for (auto line_iter = line_start; line_iter != line_end; ++line_iter) {
    if (line_iter->HasPoint()) {
      const Bin::MinZPoint& cur_point = line_iter->GetMinZPoint();
      if (cur_line_points.size() == 0) {
        int last_bin_index = 0;
        if (lines_.size() > 0) {
          last_bin_index = lines_.back().second.index.second;
        }
        if ((cur_point.z - cur_height) <
            ground_segmeter_ptr_->params_
                .initial_height_thres[cur_point.index.second -
                                      last_bin_index]) {
          cur_line_points.push_back(cur_point);
        }
      } else {
        const Bin::MinZPoint& back_point = cur_line_points.back();
        // avoid too close points
        if (cur_point.d - back_point.d > 0.05) {
          double back_height_dif = (cur_point.z - back_point.z);
          int back_index_dif = cur_point.index.second - back_point.index.second;
          if (cur_line_points.size() == 1) {
            if (back_height_dif <
                ground_segmeter_ptr_->params_.height_thres[back_index_dif]) {
              cur_line_points.push_back(cur_point);
            } else if (cur_point.z > back_point.z) {
              lines_.push_back(FitLine(cur_line_points));
              cur_height = back_point.z;
              cur_line_points.clear();
              line_iter--;
            }
          } else {
            const Bin::MinZPoint& front_point = cur_line_points.front();
            double front_height_dif = (cur_point.z - front_point.z);
            int front_index_dif =
                cur_point.index.second - front_point.index.second;
            if (back_height_dif < ground_segmeter_ptr_->params_
                                      .height_thres[back_index_dif] &&
                front_height_dif < ground_segmeter_ptr_->params_
                                       .height_thres[front_index_dif]) {
              cur_line_points.push_back(cur_point);
            } else {
              lines_.push_back(FitLine(cur_line_points));
              cur_height = lines_.back().second.z;
              cur_line_points.clear();
              line_iter--;
            }
          }
        }
      }
    }
  }
  // Add last line.
  if (cur_line_points.size() > 0) {
    lines_.push_back(FitLine(cur_line_points));
  }
}

Segment::LocalLine Segment::FitLocalLine(
    const std::list<Bin::MinZPoint>& points) {
  const unsigned int n_points = points.size();
  Eigen::MatrixXd X(n_points, 2);
  Eigen::VectorXd Y(n_points);
  unsigned int counter = 0;
  for (auto iter = points.begin(); iter != points.end(); ++iter) {
    X(counter, 0) = iter->d;
    X(counter, 1) = 1;
    Y(counter) = iter->z;
    ++counter;
  }
  Eigen::VectorXd result = X.colPivHouseholderQr().solve(Y);
  LocalLine line_result;
  line_result.first = result(0);
  line_result.second = result(1);
  return line_result;
}

Segment::Line Segment::LocalLineToLine(
    const LocalLine& local_line, const std::list<Bin::MinZPoint>& line_points) {
  Line line;
  const double first_d = line_points.front().d;
  const double second_d = line_points.back().d;
  const double first_z = local_line.first * first_d + local_line.second;
  const double second_z = local_line.first * second_d + local_line.second;
  line.first.z = first_z;
  line.first.d = first_d;
  line.first.index = line_points.front().index;
  line.second.z = second_z;
  line.second.d = second_d;
  line.second.index = line_points.back().index;
  return line;
}

Segment::Line Segment::FitLine(const std::list<Bin::MinZPoint>& points) {
  Line line;
  if (points.size() == 1) {
    const Bin::MinZPoint& back_point = points.back();
    int bin_index = back_point.index.second;
    line.first.z = back_point.z;
    line.first.d =
        (ground_segmeter_ptr_->params_.roi_r_max -
         ground_segmeter_ptr_->params_.roi_r_min) *
            pow(bin_index * 1.0 / ground_segmeter_ptr_->params_.n_bins,
                1.0 / ground_segmeter_ptr_->params_.gamma_rate) +
        ground_segmeter_ptr_->params_.roi_r_min;
    line.first.index = back_point.index;
    bin_index++;
    line.second.z = back_point.z;
    line.second.d =
        (ground_segmeter_ptr_->params_.roi_r_max -
         ground_segmeter_ptr_->params_.roi_r_min) *
            pow(bin_index * 1.0 / ground_segmeter_ptr_->params_.n_bins,
                1.0 / ground_segmeter_ptr_->params_.gamma_rate) +
        ground_segmeter_ptr_->params_.roi_r_min;
    line.second.index = back_point.index;
  } else if (points.size() > 1) {
    const LocalLine& new_line = FitLocalLine(points);
    line = LocalLineToLine(new_line, points);
  }
  return line;
}

double Segment::VerticalDistanceToLine(const double& d, const double& z,
                                       const int& bin_index) {
  double distance = std::numeric_limits<double>::max();
  for (auto it = lines_.begin(); it != lines_.end(); ++it) {
    if (it->first.index.second <= bin_index &&
        it->second.index.second >= bin_index) {
      const double delta_z = it->second.z - it->first.z;
      const double delta_d = it->second.d - it->first.d;
      const double expected_z =
          ((d - it->first.d) / delta_d) * delta_z + it->first.z;
      distance = z - expected_z;
      break;
    }
  }
  return distance;
}

LineFittingGroundSegmenter::LineFittingGroundSegmenter(
    const LineFittingParams& params) {
  segment_step_ = 2 * M_PI / params.n_segments;
  params_ = params;
  params_.initial_height_thres.resize(params_.n_bins);
  params_.height_thres.resize(params_.n_bins);
  params_.dis_to_line_thres.resize(params_.n_bins);
  params_.initial_height_thres[0] = 1.00 * params_.max_dist_to_line;
  params_.height_thres[0] = 0.50 * params_.max_dist_to_line;
  params_.dis_to_line_thres[0] = params_.max_dist_to_line;
  double initial_delta = params_.max_initial_slope;
  double delta = params_.max_slope;
  double dis_delta = 0.005;
  for (int index = 1; index < params_.n_bins; index++) {
    if (index % 8 == 0) {
      initial_delta *= 0.5;
      delta *= 0.5;
    }
    params_.initial_height_thres[index] =
        params_.initial_height_thres[index - 1] + initial_delta;
    params_.height_thres[index] = params_.height_thres[index - 1] + delta;
    params_.dis_to_line_thres[index] =
        params_.dis_to_line_thres[index - 1] + dis_delta;
  }
}

void LineFittingGroundSegmenter::Segmenter(const PointTypeCloud& cloud_in,
                                           PointTypeCloud& cloud_out) {
  points_flag_.clear();
  points_flag_.resize(cloud_in.size(), NOT_KNOWN);
  segment_coordinates_.clear();
  segment_coordinates_.resize(cloud_in.size());
  segments_.clear();
  segments_.resize(params_.n_segments, Segment(this));
  InsertPoints(cloud_in);
  GetLines();
  AssignCluster(cloud_in, cloud_out);
}

void LineFittingGroundSegmenter::InsertPoints(const PointTypeCloud& cloud) {
  for (int index = 0; index < cloud.size(); index++) {
    const PointType& point = cloud[index];
    const double range_square = point.x * point.x + point.y * point.y;
    const double range = sqrt(range_square);
    if (range < params_.roi_r_max) {
      if (range <= params_.roi_r_min) {
        points_flag_[index] = OBSTACLE;
        continue;
      }
      const double angle = std::atan2(point.y, point.x);
      int bin_index = floor((pow((range - params_.roi_r_min) /
                                     (params_.roi_r_max - params_.roi_r_min),
                                 params_.gamma_rate) *
                             params_.n_bins));
      int segment_index = floor((angle + M_PI) / segment_step_);
      segment_index =
          ((segment_index >= params_.n_segments) ? 0 : segment_index);
      segment_index = ((segment_index < 0) ? 0 : segment_index);
      std::pair<int, int> cur_index(segment_index, bin_index);
      segments_[segment_index][bin_index].AddPoint(range, point.z, cur_index);
      segment_coordinates_[index] = Bin::MinZPoint(range, point.z, cur_index);
      if (bin_index < segments_[segment_index].start_bin_index_) {
        segments_[segment_index].start_bin_index_ = bin_index;
      }
      if (bin_index > segments_[segment_index].end_bin_index_) {
        segments_[segment_index].end_bin_index_ = bin_index;
      }
    } else {
      // noise
      points_flag_[index] = NOISE;
    }
  }
}

void LineFittingGroundSegmenter::GetLines() {
  for (int index = 0; index < params_.n_segments; index++) {
    segments_[index].FitSegmentLines();
  }
}

void LineFittingGroundSegmenter::AssignCluster(const PointTypeCloud& cloud_in,
                                               PointTypeCloud& cloud_out) {
  for (int index = 0; index < points_flag_.size(); index++) {
    if (points_flag_[index] == NOT_KNOWN) {
      Bin::MinZPoint point_2d = segment_coordinates_[index];
      const int segment_index = segment_coordinates_[index].index.first;
      const int bin_index = segment_coordinates_[index].index.second;
      double dist_threshold = params_.dis_to_line_thres[bin_index];
      double dist = segments_[segment_index].VerticalDistanceToLine(
          point_2d.d, point_2d.z, point_2d.index.second);
      // Search neighboring segments.
      int steps = 1;
      while (dist >= dist_threshold &&
             steps <= params_.line_search_segment_num) {
        // Fix indices that are out of bounds.
        int index_1 = segment_index + steps;
        while (index_1 >= params_.n_segments) index_1 -= params_.n_segments;
        // Get distance to neighboring lines.
        const double dist_1 = segments_[index_1].VerticalDistanceToLine(
            point_2d.d, point_2d.z, point_2d.index.second);
        // Select smaller distance if both segments return a valid distance.
        if (dist_1 != std::numeric_limits<double>::max() && dist_1 < dist) {
          dist = dist_1;
          if (dist < dist_threshold) break;
        }
        // The same steps as index_1
        int index_2 = segment_index - steps;
        while (index_2 < 0) index_2 += params_.n_segments;
        const double dist_2 = segments_[index_2].VerticalDistanceToLine(
            point_2d.d, point_2d.z, point_2d.index.second);
        if (dist_2 != std::numeric_limits<double>::max() && dist_2 < dist) {
          dist = dist_2;
          if (dist < dist_threshold) break;
        }
        ++steps;
      }
      if (dist >= dist_threshold) {
        points_flag_[index] = OBSTACLE;
      }
    }
  }
  std::vector<int> obstacle_indices;
  obstacle_indices.reserve(points_flag_.size());
  for (int index = 0; index < points_flag_.size(); index++) {
    if (points_flag_[index] == OBSTACLE) {
      obstacle_indices.push_back(index);
    }
  }

  common::CopyPointCloud(cloud_in, obstacle_indices, cloud_out);
}
