//
// Created by Chen Xiaofeng on 19-11-1.
//

#include "reference_line.h"

namespace planning
{
  ReferenceLine::ReferenceLine(int reference_line_id)
  {
    reference_line_id_ = reference_line_id;
  }

  ReferenceLine::~ReferenceLine()
  {
  }

  void ReferenceLine::BuildLane()
  {
    lane_.id = "reference";
    lane_.type = hdmap::Lane::LaneType::NORMAL_ROAD;
    lane_.turn = hdmap::Lane::LaneTurn::NO_TURN;
    lane_.central_curve.create();
    hdmap::CurveSegment curve;
    curve.line.create();
    for (const auto &pt : central_line_.points())
    {
      common::PointENU pt_temp;
      pt_temp.x = pt.x;
      pt_temp.y = pt.y;
      curve.line->points.emplace_back(pt_temp);
    }
    lane_.central_curve->segments.emplace_back(curve);
    lane_info_.reset(new hdmap::LaneInfo(lane_));
  }

  void ReferenceLine::GetUTM(const double &accumulate_s, const double &lateral, double *utm_x, double *utm_y)
  {
    lane_info_->GetUTM(accumulate_s, lateral, utm_x, utm_y);
  }

  void ReferenceLine::GetFrenet(const double &utm_x, const double &utm_y, const double &heading, double *accumulate_s, double *lateral)
  {
    common::math::Vec2d point(utm_x, utm_y);
    lane_info_->GetProjectionWithHeading(point, heading, accumulate_s, lateral);
  }

  void ReferenceLine::GetFrenet(const double &utm_x, const double &utm_y, double *accumulate_s, double *lateral)
  {
    common::math::Vec2d point(utm_x, utm_y);
    lane_info_->GetProjection(point, accumulate_s, lateral);
  }

  bool ReferenceLine::CreateCentralTrajectory(const std::vector<TrajectoryPoint> &trajectory, const double referenceline_speed_)
  {
    if (trajectory.size() < 3)
    {
      // AERROR << "path point list size should be >= 3 ";
      return false;
    }
    central_line_.CreateTrajectory(trajectory, referenceline_speed_);
    expected_speed_ = referenceline_speed_;
    BuildLane();
  }

  bool ReferenceLine::RecordLaneIds(const std::vector<std::string> &lane_ids)
  {
    for (const auto &lane_id : lane_ids)
    {
      lane_ids_.push_back(lane_id);
    }
  }

  double ReferenceLine::Heading(const double s)
  {
    return lane_info_->Heading(s);
  }

  void ReferenceLine::SpineByFourPoints(std::vector<TrajectoryPoint> &raw_four_points_, std::vector<TrajectoryPoint> &smoother_four_points_)
  {
    auto start_point = raw_four_points_[0];
    auto end_point = raw_four_points_[3];
    //坐标系转换参数
    double diff_x = end_point.x - start_point.x;
    double diff_y = end_point.y - start_point.y;
    double diff_distance = std::hypot(diff_y, diff_x);
    double route_phi = atan2(diff_y, diff_x);
    double x_shift = start_point.x;
    double y_shift = start_point.y;
    std::vector<double> ptsx;
    std::vector<double> ptsy;
    //从全局坐标转换到局部坐标
    for (int point_count = 0; point_count < 4; ++point_count)
    {
      double x_world = raw_four_points_[point_count].x;
      double y_world = raw_four_points_[point_count].y;
      double temp_x = (x_world - x_shift) * cos(route_phi) + (y_world - y_shift) * sin(route_phi);
      double temp_y = (y_world - y_shift) * cos(route_phi) - (x_world - x_shift) * sin(route_phi);
      // AINFO << temp_x << "|" << temp_y;
      ptsx.emplace_back(temp_x);
      ptsy.emplace_back(temp_y);
    }
    tk::spline spline_tool;
    spline_tool.set_points(ptsx, ptsy);
    double x_add_on = 0;
    double spline_offset = diff_distance / 4;
    //插值并转回全局坐标系
    for (int spline_count = 0; spline_count < 4; ++spline_count)
    {
      double spline_x = x_add_on;
      double spline_y = spline_tool(spline_x);
      TrajectoryPoint temp_point;
      temp_point.x = spline_x * cos(route_phi) - spline_y * sin(route_phi) + x_shift;
      temp_point.y = spline_y * cos(route_phi) + spline_x * sin(route_phi) + y_shift;
      x_add_on += spline_offset;
      smoother_four_points_.emplace_back(temp_point);
    }
  }

  void ReferenceLine::GetSmoother()
  {
    // double enter_time = ros::Time::now().toSec();
    smoother_central_line_ = central_line_;
    int num_central_points = smoother_central_line_.mutable_points().size();
    for (int spline_end_index = 3; spline_end_index < num_central_points; spline_end_index += 2)
    {
      std::vector<TrajectoryPoint> raw_four_points;
      raw_four_points.emplace_back(smoother_central_line_.mutable_points()[spline_end_index - 3]);
      raw_four_points.emplace_back(smoother_central_line_.mutable_points()[spline_end_index - 2]);
      raw_four_points.emplace_back(smoother_central_line_.mutable_points()[spline_end_index - 1]);
      raw_four_points.emplace_back(smoother_central_line_.mutable_points()[spline_end_index - 0]);
      std::vector<TrajectoryPoint> smoother_four_points;
      SpineByFourPoints(raw_four_points, smoother_four_points);
      smoother_central_line_.mutable_points()[spline_end_index - 3] = smoother_four_points[0];
      smoother_central_line_.mutable_points()[spline_end_index - 2] = smoother_four_points[1];
      smoother_central_line_.mutable_points()[spline_end_index - 1] = smoother_four_points[2];
      smoother_central_line_.mutable_points()[spline_end_index - 0] = smoother_four_points[3];
      raw_four_points.clear();
    }
    // double end_time = ros::Time::now().toSec();
    // AINFO << end_time - enter_time;
  }
  const double ReferenceLine::Distance2Task(const double &utm_x, const double &utm_y)
  {
    double s, l;
    GetFrenet(utm_x, utm_y, &s, &l);
    if (s > 0)
      return s;
    else
      return MY_INF;
  }
} // namespace planning
