// Created by zhu-hu

#include "dubins_planner.h"

#include "common/params/vehicle_param.h"

namespace planning {
Trajectory* const DubinsPlanner::Plan(Frame* frame) {
  std::cout << "in dubins planner" << std::endl;
  double ros_time = ros::Time::now().toSec();
  const auto& adc_state = frame->local_view()->localization_estimate;
  const auto& start_point = frame->planning_start_point();
  //只用end_point的朝向信息
  auto end_point = frame->planning_target_point();

  grid_map_ = *(frame->local_view()->grid_map);
  dilated_grid_map_ = *(frame->local_view()->dilated_grid_map);
  fs_grid_map_ = *(frame->local_view()->fs_grid_map);

  //纵向采样的最大次数
  int count = frame->param->behavior_param_.max_lon_sample_nums;
  //横向采样的最大次数
  int max_sample_nums = frame->param->behavior_param_.max_lat_sample_nums;

  trajectories_.clear();

  hybrid_a_star::DubinsCurve dubins_curve;
  hybrid_a_star::DubinsPath dubins_path;

  double start[3] = {0, 0, 0};
  double end[3] = {};
  end[2] = end_point.theta;
  std::vector<double> direction_unit = {cos(end_point.theta),
                                        sin(end_point.theta)};

  std::vector<double> virtical_direction_unit = {-sin(end_point.theta),
                                                 cos(end_point.theta)};
  double radius = 3.0;

  int sample_index = 0;
  std::cout << "in dubins planner2" << std::endl;
  while (count > 0) {
    for (int i = 0; i < max_sample_nums; i++) {
      if (i % 2 == 0) {
        end[0] = end_point.x - (i / 2) * 1.0 * virtical_direction_unit[0];
        end[1] = end_point.y - (i / 2) * 1.0 * virtical_direction_unit[1];
        sample_index = i / 2;
      } else {
        end[0] = end_point.x + (i + 1) / 2 * 1.0 * virtical_direction_unit[0];
        end[1] = end_point.y + (i + 1) / 2 * 1.0 * virtical_direction_unit[1];
        sample_index = (i + 1) / 2;
      }

      //如果终点被占据，则跳出来
      if (EndPointOccupied(end) == true) continue;

      dubins_curve.dubins_shortest_path(&dubins_path, start, end, radius);

      std::cout << "curve type : " << dubins_path.type << std::endl;

      dubins_curve.dubins_path_sample_many(&dubins_path, 0.1);

      if (IsTrajectoryCollison(dubins_curve) == true) continue;

      GenerateTrajectory(dubins_curve, sample_index);
    }
    std::cout << "start_x : " << end_point.x << ", start_y : " << end_point.y
              << std::endl;

    std::cout << "count : " << count << std::endl;

    if (trajectories_.size() == 0) {
      end[0] = end_point.x - 1.5 * direction_unit[0];
      end[1] = end_point.y - 1.5 * direction_unit[1];
      end_point.x = end[0];
      end_point.y = end[1];
      count--;
      max_sample_nums -= 2;
      if (end[0] * end[0] + end[1] * end[1] <= 16.0) count = 0;
    } else {
      count = 0;
    }
  }
  JudgeTrajectories();
  PubTrajs();
  std::cout << "planner time : " << 1000 * (ros::Time::now().toSec() - ros_time)
            << " ms" << std::endl;

  if (trajectories_.empty()) return nullptr;

  int min_index = trajectories_[0].sample_index_;
  int index = 0;
  for (int i = 1; i < trajectories_.size(); i++) {
    if (trajectories_[i].sample_index_ < min_index) {
      min_index = trajectories_[i].sample_index_;
      index = i;
    }
  }
  frame->derived_trajectory_ = trajectories_[index];

  return &(frame->derived_trajectory_);
}

bool DubinsPlanner::IsTrajectoryCollison(
    const hybrid_a_star::DubinsCurve& curve) {
  // return false;
  const double adc_length = vehicle_param::kLength;
  const double adc_width = vehicle_param::kWidth;
  const double adc_rear_to_back = vehicle_param::kRearToBack;
  const double grid_map_min_x = vehicle_param::kMinX;
  const double grid_map_max_x = vehicle_param::kMaxX;
  const double grid_map_min_y = vehicle_param::kMinY;
  const double grid_map_max_y = vehicle_param::kMaxY;
  const double grid_map_pixel_scale = vehicle_param::PixelScale;
  const double shift_distance = adc_length / 2 - adc_rear_to_back;
  const double lat_safe_distance = 0.2;

  if (!dilated_grid_map_.data) {
    // std::cout << "the grid map is empty!" << std::endl;
    return true;
  }

  cv::Mat mask(dilated_grid_map_.size(), CV_8UC1, cv::Scalar::all(0));
  cv::Mat roi(dilated_grid_map_.size(), CV_8UC1, cv::Scalar::all(0));
  cv::Mat grid_map_copy(dilated_grid_map_.size(), CV_8UC1, cv::Scalar::all(0));
  dilated_grid_map_.copyTo(grid_map_copy);

  auto ego_traj_pts = curve.path_points();
  TrajectoryPoint pt;
  for (const auto& ego_traj_pt : ego_traj_pts) {
    pt.x = -(ego_traj_pt.position.y + grid_map_min_y) * grid_map_pixel_scale;
    pt.y = (-ego_traj_pt.position.x + grid_map_max_x) * grid_map_pixel_scale;
    pt.theta = tf::getYaw(ego_traj_pt.orientation);

    double theta_diff = pt.theta;
    common::math::Box2d adc_box(
        {pt.x - std::sin(theta_diff) * shift_distance * grid_map_pixel_scale,
         pt.y - std::cos(theta_diff) * shift_distance * grid_map_pixel_scale},
        -MY_PI / 2 - theta_diff, adc_length * grid_map_pixel_scale,
        (adc_width + 2 * lat_safe_distance) * grid_map_pixel_scale);
    int col_min = static_cast<int>(adc_box.min_x());
    int col_max = static_cast<int>(adc_box.max_x());
    int row_min = static_cast<int>(adc_box.min_y());
    int row_max = static_cast<int>(adc_box.max_y());

    for (int i = row_min; i <= row_max; ++i) {
      for (int j = col_min; j <= col_max; ++j) {
        if (j > 0 && j < dilated_grid_map_.cols && i > 0 &&
            i < dilated_grid_map_.rows) {
          if (adc_box.IsPointIn({j + 0.5, i + 0.5})) mask.at<uchar>(i, j) = 255;
        }
      }
    }

    grid_map_copy.copyTo(roi, mask);
    int occupied = countNonZero(roi);

    if (occupied > 0) {
      // cv::imshow("roi", roi);
      // cv::imshow("mask", mask);
      // cv::waitKey(1);
      // nearest_collision_point_s = pt.s;
      return true;
    }
  }

  // nearest_collision_point_s = std::numeric_limits<double>::max();
  return false;
}

void DubinsPlanner::GenerateTrajectory(const hybrid_a_star::DubinsCurve& curve,
                                       int index) {
  Trajectory traj;
  std::vector<TrajectoryPoint> points;
  for (const auto& point : curve.path_points()) {
    TrajectoryPoint pt;
    pt.x = point.position.x;
    pt.y = point.position.y;
    points.emplace_back(pt);
  }
  traj.CreateTrajectory(points, index);

  trajectories_.emplace_back(traj);
}

void DubinsPlanner::JudgeTrajectories() {}

bool DubinsPlanner::EndPointOccupied(const double* end) {
  const double adc_length = vehicle_param::kLength;
  const double adc_width = vehicle_param::kWidth;
  const double adc_rear_to_back = vehicle_param::kRearToBack;
  const double grid_map_min_x = vehicle_param::kMinX;
  const double grid_map_max_x = vehicle_param::kMaxX;
  const double grid_map_min_y = vehicle_param::kMinY;
  const double grid_map_max_y = vehicle_param::kMaxY;
  const double grid_map_pixel_scale = vehicle_param::PixelScale;
  const double shift_distance = adc_length / 2 - adc_rear_to_back;
  const double lat_safe_distance = 0.5;

  if (!fs_grid_map_.data) {
    // std::cout << "the grid map is empty!" << std::endl;
    return true;
  }

  if (end[0] > grid_map_max_x || end[0] < grid_map_min_x ||
      end[1] > grid_map_max_y || end[1] < grid_map_min_y) {
    return true;
  }

  cv::Mat mask(fs_grid_map_.size(), CV_8UC1, cv::Scalar::all(0));
  cv::Mat roi(fs_grid_map_.size(), CV_8UC1, cv::Scalar::all(0));
  cv::Mat grid_map_copy(fs_grid_map_.size(), CV_8UC1, cv::Scalar::all(0));
  fs_grid_map_.copyTo(grid_map_copy);

  TrajectoryPoint pt;
  double adc_theta = end[2];

  pt.x = -(end[1] + grid_map_min_y) * grid_map_pixel_scale;
  pt.y = (-end[0] + grid_map_max_x) * grid_map_pixel_scale;
  pt.theta = end[2];

  double theta_diff = pt.theta - adc_theta;
  common::math::Box2d adc_box(
      {pt.x - std::sin(theta_diff) * shift_distance * grid_map_pixel_scale,
       pt.y - std::cos(theta_diff) * shift_distance * grid_map_pixel_scale},
      -MY_PI / 2 - theta_diff, adc_length * grid_map_pixel_scale,
      (adc_width + 2 * lat_safe_distance) * grid_map_pixel_scale);
  int col_min = static_cast<int>(adc_box.min_x());
  int col_max = static_cast<int>(adc_box.max_x());
  int row_min = static_cast<int>(adc_box.min_y());
  int row_max = static_cast<int>(adc_box.max_y());

  for (int i = row_min; i <= row_max; ++i) {
    for (int j = col_min; j <= col_max; ++j) {
      if (j > 0 && j < fs_grid_map_.cols && i > 0 && i < fs_grid_map_.rows) {
        if (adc_box.IsPointIn({j + 0.5, i + 0.5})) mask.at<uchar>(i, j) = 255;
      }
    }
  }

  grid_map_copy.copyTo(roi, mask);
  int occupied = countNonZero(roi);

  if (occupied > 0) {
    return true;
  }
  return false;
}

void DubinsPlanner::PubTrajs() {
  int id = 0;
  visualization_msgs::MarkerArray trajectories_visualization;
  if (trajectories_.empty()) return;
  int min_index = trajectories_[0].sample_index_;
  int index = 0;
  for (int i = 1; i < trajectories_.size(); i++) {
    if (trajectories_[i].sample_index_ < min_index) {
      min_index = trajectories_[i].sample_index_;
      index = i;
    }
  }
  int count = 0;
  for (const auto& derived_trajectory : trajectories_) {
    for (size_t i = 0; i < derived_trajectory.points().size() - 1; i++) {
      visualization_msgs::Marker line;
      line.header.stamp = ros::Time::now();
      line.header.frame_id = "base_link";
      line.action = visualization_msgs::Marker::ADD;
      line.type = visualization_msgs::Marker::LINE_LIST;
      std_msgs::ColorRGBA color;
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      color.a = 1.0;
      if (derived_trajectory.points()[i].velocity >= 0.0) {
        line.color = color;
      } else {
        line.color.a = 1.0;
        line.color.r = 1.0;
        line.color.g = 0.0;
        line.color.b = 0.0;
      }
      //选择index最小的，用粗绿色线条画出来
      if (count == index) {
        line.scale.x = 0.3;
        line.scale.y = 0.3;
        line.color.a = 1.0;
        line.color.r = 0.0;
        line.color.g = 0.0;
        line.color.b = 1.0;
      } else {
        line.scale.x = 0.1;
        line.scale.y = 0.1;
      }
      line.lifetime = ros::Duration(0.2);
      line.id = id++;
      line.points.emplace_back();
      line.points.back().x = derived_trajectory.points()[i].x;
      line.points.back().y = derived_trajectory.points()[i].y;
      line.points.back().z = 0.0;

      line.id = id++;
      line.points.emplace_back();
      line.points.back().x = derived_trajectory.points()[i + 1].x;
      line.points.back().y = derived_trajectory.points()[i + 1].y;
      line.points.back().z = 0.0;
      trajectories_visualization.markers.emplace_back(line);
    }
    count++;
    // z += 1.0;
    // visualization_msgs::Marker cost_vis;
    // cost_vis.id = id++;
    // cost_vis.header.stamp = ros::Time::now();
    // cost_vis.header.frame_id = "world";
    // cost_vis.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // cost_vis.action = visualization_msgs::Marker::ADD;
    // cost_vis.scale.x = 1;
    // cost_vis.scale.y = 1;
    // cost_vis.scale.z = 1;
    // cost_vis.pose.position.x = derived_trajectory.points().back().x;
    // cost_vis.pose.position.y = derived_trajectory.points().back().y;
    // cost_vis.pose.position.z = 0.0;
    // cost_vis.pose.orientation.x = 0.0;
    // cost_vis.pose.orientation.y = 0.0;
    // cost_vis.pose.orientation.z = 0.0;
    // cost_vis.pose.orientation.w = 1.0;
    // cost_vis.scale.z = 1;
    // cost_vis.color.r = 1.0f;
    // cost_vis.color.g = 0.0f;
    // cost_vis.color.b = 1.0f;
    // cost_vis.color.a = 1.0;
    // int cost = derived_trajectory.GetCost();
    // std::string text = std::to_string(cost);
    // cost_vis.text = text;
    // cost_vis.lifetime = ros::Duration(0.2);
    // trajectories_visualization.markers.emplace_back(cost_vis);
  }
  pub_trajs_.publish(trajectories_visualization);
}
}  // namespace planning