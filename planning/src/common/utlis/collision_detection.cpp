//
// Created by wl on 2019/11/7.
//

#include "collision_detection.h"

namespace planning {

bool collision_detect(Trajectory *trajectory,
                      const std::vector<PredictionObstacle> &obstacles,
                      const cv::Mat &grid_map, const Parameter *param) {
  trajectory->set_lateral_distance_to_static_obstacle(
      std::numeric_limits<double>::max());
  trajectory->set_longitude_distance_to_static_obstacle(
      std::numeric_limits<double>::max());
  trajectory->set_lateral_distance_to_dynamic_obstacle(
      std::numeric_limits<double>::max());
  trajectory->set_longitude_distance_to_dynamic_obstacle(
      std::numeric_limits<double>::max());
  trajectory->set_lateral_distance_to_pedestrian(
      std::numeric_limits<double>::max());
  trajectory->set_longitude_distance_to_pedestrian(
      std::numeric_limits<double>::max());
  trajectory->set_lateral_distance_to_cone(std::numeric_limits<double>::max());
  trajectory->set_longitude_distance_to_cone(
      std::numeric_limits<double>::max());
  trajectory->set_front_dynamic_obstacle_speed(
      std::numeric_limits<double>::max());

  const double length = param->behavior_param_.vehicle_length;
  const double width = param->behavior_param_.vehicle_width;
  const double rear_to_back = param->behavior_param_.rear_to_back;
  const double shift_distance = length / 2 - rear_to_back;
  const double dilate_width =
      width + 2 * param->behavior_param_.lat_safe_distance_static_obstacle;

  auto ego_traj_pts = trajectory->points();

  bool is_colliding_with_static_obstacles = false;
  bool is_colliding_with_pedestrian = false;
  bool is_colliding_with_cone = false;
  bool is_colliding_with_dynamic_obstacles = false;

  // 动态车辆与行人碰撞检测
  for (const auto &ego_traj_pt : ego_traj_pts) {
    //从后轴到车头的障碍物不考虑
    // if (ego_traj_pt.s < (length - rear_to_back))
    // {
    //   continue;
    // }

    double ego_center_x =
        ego_traj_pt.x + (length / 2.0 - rear_to_back) * cos(ego_traj_pt.theta);
    double ego_center_y =
        ego_traj_pt.y + (length / 2.0 - rear_to_back) * sin(ego_traj_pt.theta);
    Vec2d ego_center(ego_center_x, ego_center_y);

    Box2d ego_box2d(ego_center, ego_traj_pt.theta, length, width);
    Polygon2d ego_polygon2d(ego_box2d);

    double longitude_distance_to_dynamic_obstacle =
        std::numeric_limits<double>::max();

    for (const auto &obstacle : obstacles) {
      if (obstacle.polygon2d.num_points() < 3) {
        AERROR << "the number of points of polygen is small than 3!";
        continue;
      }

      Polygon2d obstacle_polygon2d(obstacle.polygon2d.points());
      //将速度投影到轨迹的方向
      double obstacle_velocity =
          obstacle.velocity *
          cos(normalize_angle(obstacle.theta - ego_traj_pt.theta));

      // 行人
      if (obstacle.type == PEDESTRIAN) {
        if (ego_polygon2d.HasOverlap(obstacle_polygon2d)) {
          trajectory->set_lateral_distance_to_pedestrian(0.0);
          trajectory->set_longitude_distance_to_pedestrian(std::min(
              ego_traj_pt.s, trajectory->longitude_distance_to_pedestrian()));

          is_colliding_with_pedestrian = true;
        }
      }

      // 锥桶
      if (obstacle.type == CONE) {
        if (ego_polygon2d.HasOverlap(obstacle_polygon2d)) {
          trajectory->set_lateral_distance_to_cone(0.0);
          trajectory->set_longitude_distance_to_cone(std::min(
              ego_traj_pt.s, trajectory->longitude_distance_to_cone()));
          trajectory->set_lateral_distance_to_static_obstacle(0.0);
          trajectory->set_longitude_distance_to_static_obstacle(
              std::min(ego_traj_pt.s,
                       trajectory->longitude_distance_to_static_obstacles()));

          is_colliding_with_static_obstacles = true;
          is_colliding_with_cone = true;
        }
      }

      // 动态车辆
      if (obstacle.type == UNKNOWN || CAR) {
        if (ego_polygon2d.HasOverlap(obstacle_polygon2d)) {
          // 不超动态，动态是多少速度就赋多少速度
          if (longitude_distance_to_dynamic_obstacle > ego_traj_pt.s) {
            longitude_distance_to_dynamic_obstacle = ego_traj_pt.s;

            if (longitude_distance_to_dynamic_obstacle <
                trajectory->longitude_distance_to_dynamic_obstacles()) {
              trajectory->set_front_dynamic_obstacle(obstacle);
              trajectory->set_lateral_distance_to_dynamic_obstacle(0.0);
              trajectory->set_longitude_distance_to_dynamic_obstacle(
                  longitude_distance_to_dynamic_obstacle);

              // 不要给太大的负速度不然减速会很猛
              if (obstacle_velocity > -2) {
                trajectory->set_front_dynamic_obstacle_speed(obstacle_velocity);
              } else {
                trajectory->set_front_dynamic_obstacle_speed(-2);
              }

              is_colliding_with_dynamic_obstacles = true;
            }
          }
        }
      }
    }
  }

  //静态障碍物碰撞检测
  const double grid_map_min_x = param->grid_map_param_.min_x;
  const double grid_map_max_x = param->grid_map_param_.max_x;
  const double grid_map_min_y = param->grid_map_param_.min_y;
  const double grid_map_max_y = param->grid_map_param_.max_y;
  const double grid_map_pixel_scale = param->grid_map_param_.pixel_scale;

  if (!grid_map.data) {
    // std::cout << "the grid map is empty!" << std::endl;
    return true;
  }

  cv::Mat mask(grid_map.size(), CV_8UC1, cv::Scalar::all(0));
  cv::Mat roi(grid_map.size(), CV_8UC1, cv::Scalar::all(0));
  cv::Mat grid_map_copy(grid_map.size(), CV_8UC1, cv::Scalar::all(0));
  grid_map.copyTo(grid_map_copy);

  static tf::TransformListener listener;
  geometry_msgs::PointStamped pt_in_world;
  geometry_msgs::PointStamped pt_in_car;
  TrajectoryPoint pt;
  double adc_theta = ego_traj_pts.front().theta;

  for (const auto &ego_traj_pt : ego_traj_pts) {
    // cv::Mat mask(grid_map.size(), CV_8UC1, cv::Scalar::all(0));
    Vec2d traj_pt, pt_in_car;
    traj_pt.set_x(ego_traj_pt.x);
    traj_pt.set_y(ego_traj_pt.y);
    global_to_local(traj_pt, pt_in_car);

    pt.x = -(pt_in_car.y() + grid_map_min_y) * grid_map_pixel_scale;
    pt.y = (-pt_in_car.x() + grid_map_max_x) * grid_map_pixel_scale;
    pt.theta = ego_traj_pt.theta;
    pt.s = ego_traj_pt.s;
    pt.l = ego_traj_pt.l;

    double theta_diff = pt.theta - adc_theta;
    common::math::Box2d adc_box(
        {pt.x - std::sin(theta_diff) * shift_distance * grid_map_pixel_scale,
         pt.y - std::cos(theta_diff) * shift_distance * grid_map_pixel_scale},
        -MY_PI / 2 - theta_diff, length * grid_map_pixel_scale,
        dilate_width * grid_map_pixel_scale);
    int col_min = static_cast<int>(adc_box.min_x());
    int col_max = static_cast<int>(adc_box.max_x());
    int row_min = static_cast<int>(adc_box.min_y());
    int row_max = static_cast<int>(adc_box.max_y());

    for (int i = row_min; i <= row_max; ++i) {
      for (int j = col_min; j <= col_max; ++j) {
        if (j > 0 && j < grid_map.cols && i > 0 && i < grid_map.rows) {
          if (adc_box.IsPointIn({j + 0.5, i + 0.5})) mask.at<uchar>(i, j) = 255;
        }
      }
    }

    grid_map_copy.copyTo(roi, mask);
    int occupied = countNonZero(roi);
    // std::vector<cv::Point> occupied_grids;
    // findNonZero(roi, occupied_grids);
    if (occupied) {
      trajectory->set_longitude_distance_to_static_obstacle(
          std::min(pt.s, trajectory->longitude_distance_to_static_obstacles()));
      trajectory->set_lateral_distance_to_static_obstacle(0.0);
      is_colliding_with_static_obstacles = true;
      break;
    }
  }

  // cv::imshow("ROI", roi);
  // cv::imshow("GRID_MAP", grid_map_test);
  // cv::imshow("MASK", mask);
  // cv::waitKey(1);

  return is_colliding_with_static_obstacles ||
         is_colliding_with_dynamic_obstacles || is_colliding_with_pedestrian ||
         is_colliding_with_cone;
}

bool collision_detect_for_unprotected_turn(
    const std::vector<PredictionObstacle> &obstacles,
    const Polygon2d &detection_area, const double &ego_yaw) {
  // 速度阈值，低于Xm/s的障碍物不考虑
  const double velocity_threshold = 1;
  // 角度阈值，考虑与自车夹角小于X度的障碍物
  const double angle_threshold = MY_PI / 4;

  for (const auto &obstacle : obstacles) {
    if (obstacle.polygon2d.num_points() < 3) {
      AERROR << "the number of points of polygen is small than 3!";
      continue;
    }

    if (obstacle.velocity < velocity_threshold) {
      continue;
    }

    Polygon2d obstacle_polygon2d(obstacle.polygon2d.points());
    if (detection_area.HasOverlap(obstacle_polygon2d) &&
        abs(obstacle.theta - ego_yaw) < MY_PI / 4) {
      return true;
    }
  }

  return false;
}

bool collision_detect_for_sidewalk(
    const std::vector<PredictionObstacle> &obstacles,
    const Polygon2d &sidewalk_detection_area) {
  // 速度阈值，低于Xm/s的障碍物不考虑
  const double velocity_threshold = 0.3;

  for (const auto &obstacle : obstacles) {
    if (obstacle.polygon2d.num_points() < 3) {
      AERROR << "the number of points of polygen is small than 3!";
      continue;
    }

    if (obstacle.velocity < velocity_threshold) {
      continue;
    }

    if (obstacle.type == PEDESTRIAN) {
      Polygon2d obstacle_polygon2d(obstacle.polygon2d.points());
      if (sidewalk_detection_area.HasOverlap(obstacle_polygon2d)) {
        return true;
      }
    }
  }

  return false;
}

bool collision_detect_for_cautious_zone(
    const std::vector<PredictionObstacle> &obstacles,
    const Polygon2d &cautious_zone_area, const cv::Mat &grid_map,
    const Parameter *param) {
  // 速度阈值，低于Xm/s的障碍物不考虑
  const double velocity_threshold = 0.3;

  for (const auto &obstacle : obstacles) {
    if (obstacle.polygon2d.num_points() < 3) {
      AERROR << "the number of points of polygen is small than 3!";
      continue;
    }

    if (obstacle.velocity < velocity_threshold) {
      continue;
    }

    if (obstacle.type == PEDESTRIAN || CAR || UNKNOWN) {
      Polygon2d obstacle_polygon2d(obstacle.polygon2d.points());
      if (cautious_zone_area.HasOverlap(obstacle_polygon2d)) {
        return true;
      }
    }
  }

  //静态障碍物碰撞检测
  const double grid_map_min_x = param->grid_map_param_.min_x;
  const double grid_map_max_x = param->grid_map_param_.max_x;
  const double grid_map_min_y = param->grid_map_param_.min_y;
  const double grid_map_max_y = param->grid_map_param_.max_y;
  const double grid_map_pixel_scale = param->grid_map_param_.pixel_scale;

  if (!grid_map.data) {
    // std::cout << "the grid map is empty!" << std::endl;
    return true;
  }

  cv::Mat mask(grid_map.size(), CV_8UC1, cv::Scalar::all(0));
  cv::Mat roi(grid_map.size(), CV_8UC1, cv::Scalar::all(0));
  cv::Mat grid_map_copy(grid_map.size(), CV_8UC1, cv::Scalar::all(0));
  grid_map.copyTo(grid_map_copy);

  std::vector<Vec2d> pts_in_map;
  for (const auto &polygon2d_pt : cautious_zone_area.points()) {
    Vec2d pt_in_local;
    global_to_local(polygon2d_pt, pt_in_local);

    Vec2d pt_in_map;
    pt_in_map.set_x(-(pt_in_local.y() + grid_map_min_y) * grid_map_pixel_scale);
    pt_in_map.set_y((-pt_in_local.x() + grid_map_max_x) * grid_map_pixel_scale);
    pts_in_map.emplace_back(pt_in_map);
  }

  Polygon2d polygon2d_in_map(pts_in_map);
  int col_min = static_cast<int>(polygon2d_in_map.min_x());
  int col_max = static_cast<int>(polygon2d_in_map.max_x());
  int row_min = static_cast<int>(polygon2d_in_map.min_y());
  int row_max = static_cast<int>(polygon2d_in_map.max_y());
  for (int i = row_min; i <= row_max; ++i) {
    for (int j = col_min; j <= col_max; ++j) {
      if (j > 0 && j < grid_map.cols && i > 0 && i < grid_map.rows) {
        if (polygon2d_in_map.IsPointIn({j + 0.5, i + 0.5}))
          mask.at<uchar>(i, j) = 255;
      }
    }
  }

  grid_map_copy.copyTo(roi, mask);
  int occupied = countNonZero(roi);
  if (occupied) return true;

  return false;
}

bool collision_detect_for_lane_change(
    const std::vector<PredictionObstacle> &obstacles,
    const Polygon2d &lane_change_detection_area,
    const double lane_change_ignore_oncoming_vehicles_velocity_threshold) {
  for (const auto &obstacle : obstacles) {
    if (obstacle.polygon2d.num_points() < 3) {
      AERROR << "the number of points of polygen is small than 3!";
      continue;
    }

    if (obstacle.velocity <
        lane_change_ignore_oncoming_vehicles_velocity_threshold) {
      continue;
    }

    if (obstacle.type == UNKNOWN) {
      Polygon2d obstacle_polygon2d(obstacle.polygon2d.points());
      if (lane_change_detection_area.HasOverlap(obstacle_polygon2d)) {
        AINFO << "obs vel:  " << obstacle.velocity;
        return true;
      }
    }
  }

  return false;
}

bool global_to_local(const Vec2d &global_pt, Vec2d &local_pt) {
  static tf::TransformListener listener;

  geometry_msgs::PointStamped pt_in_world;
  geometry_msgs::PointStamped pt_in_car;

  pt_in_world.header.frame_id = "world";
  pt_in_world.point.x = global_pt.x();
  pt_in_world.point.y = global_pt.y();
  pt_in_world.point.z = 0;

  try {
    listener.transformPoint("base_link", pt_in_world, pt_in_car);
  } catch (tf::TransformException ex) {
    return false;
  }

  local_pt.set_x(pt_in_car.point.x);
  local_pt.set_y(pt_in_car.point.y);

  return true;
}

// bool collision_detect(Trajectory *trajectory, const
// std::vector<PredictionObstacle> &obstacles)
// {
//   trajectory->set_lateral_distance_to_static_obstacle(std::numeric_limits<double>::max());
//   trajectory->set_longitude_distance_to_static_obstacle(std::numeric_limits<double>::max());
//   trajectory->set_longitude_distance_to_dynamic_obstacle(std::numeric_limits<double>::max());
//   trajectory->set_lateral_distance_to_dynamic_obstacle(std::numeric_limits<double>::max());
//   trajectory->set_front_dynamic_obstacle_speed(std::numeric_limits<double>::max());

//   if (obstacles.empty())
//     return false;

//   const double length = vehicle_model::length;
//   const double width = vehicle_model::width;
//   const double rear_to_back = vehicle_model::rear_to_back;

//   auto ego_traj_pts = trajectory->points();

//   for (const auto &ego_traj_pt : ego_traj_pts)
//   {
//     if (ego_traj_pt.s < 3.0) //从后轴开始往前3m内的障碍物不考虑
//       continue;
//     double ego_center_x = ego_traj_pt.x + (length / 2.0 - rear_to_back) *
//     cos(ego_traj_pt.theta); double ego_center_y = ego_traj_pt.y + (length
//     / 2.0 - rear_to_back) * sin(ego_traj_pt.theta); Vec2d
//     ego_center(ego_center_x, ego_center_y); Box2d ego_box2d(ego_center,
//     ego_traj_pt.theta, length + 0.1, width + 0.65); Polygon2d
//     ego_polygon2d(ego_box2d);

//     for (const auto &obstacle : obstacles)
//     {
//       if (obstacle.polygon2d.num_points() < 3)
//       {
//         AERROR << "the number of points of polygen is small than 3!";
//         continue;
//       }
//       Polygon2d obstacle_polygon2d(obstacle.polygon2d.points());
//       double obstacle_velocity = obstacle.velocity *
//       cos(normalize_angle(obstacle.theta - ego_traj_pt.theta));
//       //将速度投影到轨迹的方向
//       // if(obstacle.motion_status == DYNAMIC)
//       //   AINFO<<"id: "<<obstacle.id<<" motion_status:
//       "<<obstacle.motion_status<<" speed: "<<obstacle_velocity; if
//       (ego_polygon2d.HasOverlap(obstacle_polygon2d))
//       {
//         if (obstacle.motion_status == STATIC)
//         {
//           if (ego_traj_pt.s > 40)
//           {
//             trajectory->set_longitude_distance_to_static_obstacle(ego_traj_pt.s);
//             //后面统一cut distance和提前减速，不然安全距离太远
//             trajectory->set_lateral_distance_to_static_obstacle(0.0);
//             return false;
//           }
//           trajectory->set_longitude_distance_to_static_obstacle(ego_traj_pt.s);
//           trajectory->set_lateral_distance_to_static_obstacle(0.0);
//           return true;
//         }
//         else if (obstacle.motion_status == DYNAMIC)
//         {
//           // 不超动态，动态是多少速度就赋多少速度
//           trajectory->set_front_dynamic_obstacle(obstacle);
//           trajectory->set_longitude_distance_to_dynamic_obstacle(ego_traj_pt.s);
//           trajectory->set_lateral_distance_to_dynamic_obstacle(0.0);
//           // 不要给太大的负速度不然减速会很猛
//           if (obstacle_velocity > -2)
//             trajectory->set_front_dynamic_obstacle_speed(obstacle_velocity);
//           else
//             trajectory->set_front_dynamic_obstacle_speed(-2);
//           // if (obstacle_velocity < 2.0 && obstacle.type != PEDESTRIAN)
//           //是否超车的速度临界
//           // {
//           //   if (ego_traj_pt.s > 40)
//           //   {
//           //
//           trajectory->set_longitude_distance_to_dynamic_obstacle(ego_traj_pt.s);
//           //后面统一cut distance，不然安全距离太远
//           //     trajectory->set_lateral_distance_to_dynamic_obstacle(0.0);
//           //     return false;
//           //   }
//           //
//           trajectory->set_longitude_distance_to_dynamic_obstacle(ego_traj_pt.s);
//           //   trajectory->set_lateral_distance_to_dynamic_obstacle(0.0);
//           //   return true;
//           // }
//           return false;
//         }
//       }
//     }
//   }
//   return false;
// }

// bool collision_detect_for_rrt(Trajectory *trajectory, const
// std::vector<PredictionObstacle> &obstacles)
// {
//   trajectory->set_lateral_distance_to_static_obstacle(std::numeric_limits<double>::max());
//   trajectory->set_longitude_distance_to_static_obstacle(std::numeric_limits<double>::max());
//   if (obstacles.empty())
//     return false;
//   const double length = vehicle_model::length;
//   const double width = vehicle_model::width;
//   const double rear_to_back = vehicle_model::rear_to_back;
//   auto ego_traj_pts = trajectory->points();
//   for (const auto &ego_traj_pt : ego_traj_pts)
//   {
//     if (ego_traj_pt.s < 3.0)
//       continue;
//     double ego_center_x = ego_traj_pt.x + (length / 2.0 - rear_to_back) *
//     cos(ego_traj_pt.theta); double ego_center_y = ego_traj_pt.y + (length
//     / 2.0 - rear_to_back) * sin(ego_traj_pt.theta); Vec2d
//     ego_center(ego_center_x, ego_center_y); Box2d ego_box2d(ego_center,
//     ego_traj_pt.theta, length + 0.1, width); Polygon2d
//     ego_polygon2d(ego_box2d);

//     for (const auto &obstacle : obstacles)
//     {
//       if (obstacle.polygon2d.num_points() < 3)
//       {
//         AERROR << "the number of points of polygen is small than 3!";
//         continue;
//       }
//       Polygon2d obstacle_polygon2d(obstacle.polygon2d.points());
//       double obstacle_velocity = obstacle.velocity *
//       cos(normalize_angle(obstacle.theta - ego_traj_pt.theta));
//       //将速度投影到轨迹的方向 if
//       (ego_polygon2d.HasOverlap(obstacle_polygon2d))
//       {
//         trajectory->set_longitude_distance_to_static_obstacle(ego_traj_pt.s);
//         trajectory->set_lateral_distance_to_static_obstacle(0.0);
//         return true;
//       }
//     }
//   }
//   return false;
// }

// bool collision_detect_for_trajectories(const std::vector<Trajectory>
// *trajectories_, const std::vector<PredictionObstacle> &obstacles_, const
// cv::Mat &grid_map_)
// {
//   for (auto traj : *trajectories_)
//   {
//     if (collision_detect(&traj, obstacles_, grid_map_))
//     {
//       return true;
//     }
//   }
//   return false;
// }
}  // namespace planning
