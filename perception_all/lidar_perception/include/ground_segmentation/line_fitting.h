/*
 * Copyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao Tong
 * University. All rights reserved.
 * Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.02.21
 */

#ifndef GROUND_SEGMENTATION_LINE_FITTING_H_
#define GROUND_SEGMENTATION_LINE_FITTING_H_

#include <mrpt/poses/CPose3D.h>

#include <limits>
#include <list>
#include <vector>

#include "common/point_type.h"
#include "common/util.h"

class Bin {
 public:
  struct MinZPoint {
    MinZPoint() : z(0), d(0), index(0, 0) {}
    MinZPoint(const double& d, const double& z,
              const std::pair<int, int>& index)
        : d(d), z(z), index(index) {}
    bool operator==(const MinZPoint& comp) {
      return z == comp.z && d == comp.d && index == comp.index;
    }

    double z;
    double d;
    std::pair<int, int> index;  // <segment_index, bin_index>
  };

 public:
  Bin();
  // brief Fake copy constructor to allow vector<vector<Bin> > initialization.
  Bin(const Bin& segment);
  void AddPoint(const PointType& point, const std::pair<int, int>& index);
  void AddPoint(const double& d, const double& z,
                const std::pair<int, int>& index);
  MinZPoint GetMinZPoint();
  inline bool HasPoint() { return has_point_; }

 private:
  bool has_point_;
  double min_z_;
  double min_z_range_;
  std::pair<int, int> min_z_index_;
};

class LineFittingGroundSegmenter;

class Segment {
 public:
  typedef std::pair<Bin::MinZPoint, Bin::MinZPoint> Line;
  typedef std::pair<double, double> LocalLine;
  int start_bin_index_;
  int end_bin_index_;

 public:
  Segment(LineFittingGroundSegmenter* ground_segmeter_ptr);

  double VerticalDistanceToLine(const double& d, const double& z,
                                const int& bin_index);

  void FitSegmentLines();

  inline Bin& operator[](const size_t& index) { return bins_[index]; }

  inline std::vector<Bin>::iterator begin() { return bins_.begin(); }

  inline std::vector<Bin>::iterator end() { return bins_.end(); }

 private:
  LineFittingGroundSegmenter* ground_segmeter_ptr_;

  std::vector<Bin> bins_;

  std::list<Line> lines_;

  LocalLine FitLocalLine(const std::list<Bin::MinZPoint>& points);

  Line LocalLineToLine(const LocalLine& local_line,
                       const std::list<Bin::MinZPoint>& line_points);

  Line FitLine(const std::list<Bin::MinZPoint>& points);
};

class LineFittingGroundSegmenter {
 public:
  enum point_flag { NOT_KNOWN, NOISE, GROUND, OBSTACLE };

  struct LineFittingParams {
    // Minimum range of segmentation.
    double roi_r_min = 0.40;
    // Maximum range of segmentation.
    double roi_r_max = 30.00;

    // Number of radial bins.
    int n_bins = 25;
    // Number of angular segments.
    int n_segments = 50;
    // Adjust Radial grid division. -- (0,1]
    double gamma_rate = 0.70;
    // Prior z value of ground
    double prior_ground_z = 0.00;
    // Maximum initial slope to find the first point of fitting line
    double max_initial_slope = 0.04;
    // Maximum slope to find the subsequent points of fitting line
    double max_slope = 0.02;
    // How far to search for a line in angular direction(segment num as
    // distance).
    int line_search_segment_num = 3;
    // Maximum distance to a ground line to be classified as ground.
    double max_dist_to_line = 0.25;

    // Number of threads.
    int num_threads = 4;

    std::vector<double> initial_height_thres;
    std::vector<double> height_thres;
    std::vector<double> dis_to_line_thres;
  };

  LineFittingGroundSegmenter(const LineFittingParams& params);

  LineFittingParams params_;

  // Segment the point cloud.
  void Segmenter(const PointTypeCloud& cloud_in, PointTypeCloud& cloud_out);

 private:
  double segment_step_;

  // Access with segments_[segment][bin].
  std::vector<Segment> segments_;

  // 2D coordinates (d, z) of every point in its respective segment.
  std::vector<Bin::MinZPoint> segment_coordinates_;

  std::vector<point_flag> points_flag_;

  void InsertPoints(const PointTypeCloud& cloud);

  void GetLines();

  void AssignCluster(const PointTypeCloud& cloud_in, PointTypeCloud& cloud_out);
};

#endif  // GROUND_SEGMENTATION_LINE_FITTING_H_
