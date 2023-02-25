#include "osm_route_manager.h"

namespace planning {
OsmRouteManager::OsmRouteManager(ros::NodeHandle* nh, Parameter* param,
                                 OsmMapManager* osm_map_manager)
    : nh_(nh), param_(param), osm_map_manager_(osm_map_manager) {
  pub_route_path_ = nh_->advertise<visualization_msgs::MarkerArray>(
      "/planning/route_path", 5, this);
  pub_smoothed_route_rviz_ = nh_->advertise<visualization_msgs::MarkerArray>(
      "/planning/smoothed_route_path", 5, this);
  sub_target_point_ =
      nh_->subscribe("/planning/target_point_id", 5,
                     &OsmRouteManager::TargetPointCallback, this);
  sub_move_base_goal_ =
      nh_->subscribe("/move_base_simple/goal", 5,
                     &OsmRouteManager::MoveBaseGoalCallback, this);
  smoothed_route_path_ = param_->behavior_param_.smoothed_route_path;
  listener_ = new tf::TransformListener();
}

OsmRouteManager::~OsmRouteManager() {}

void OsmRouteManager::PublishRoutePath() {
  if (!path_found_) return;
  visualization_msgs::MarkerArray route_path_markers;
  visualization_msgs::Marker marker;
  int id_count = 0;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "path_points";
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.lifetime = ros::Duration(0.2);
  // Scale
  marker.scale.x = 3.8;
  marker.scale.y = 1.8;
  marker.scale.z = 1.8;
  // Color
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  for (int i = 0; i < path_ids_.size() - 1; i++) {
    if (path_ids_[i] == current_navigation_node_id_) {
      marker.scale.x = 5.8;
      marker.scale.y = 3.8;
      marker.scale.z = 3.8;
      // Color
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
    } else {
      // Scale
      marker.scale.x = 3.8;
      marker.scale.y = 1.8;
      marker.scale.z = 1.8;
      // Color
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }
    marker.id = id_count++;
    // marker.color.a = 1.0 * (i + 1) / (path_ids_.size() + 1);
    marker.color.a = 1.0;
    marker.pose.position.x =
        osm_map_manager_->osm_map()->osm_nodes()[path_ids_[i]].utm_x;
    marker.pose.position.y =
        osm_map_manager_->osm_map()->osm_nodes()[path_ids_[i]].utm_y;
    marker.pose.position.z = 0.0;
    // double delta_x =
    //     osm_map_manager_->osm_map()->osm_nodes()[path_ids_[i + 1]].utm_x -
    //     osm_map_manager_->osm_map()->osm_nodes()[path_ids_[i]].utm_x;
    // double delta_y =
    //     osm_map_manager_->osm_map()->osm_nodes()[path_ids_[i + 1]].utm_y -
    //     osm_map_manager_->osm_map()->osm_nodes()[path_ids_[i]].utm_y;
    // double yaw = atan2(delta_y, delta_x);
    double yaw = path_point_theta_[i];
    auto tf_q = tf::createQuaternionFromYaw(yaw);
    marker.pose.orientation.x = tf_q.getX();
    marker.pose.orientation.y = tf_q.getY();
    marker.pose.orientation.z = tf_q.getZ();
    marker.pose.orientation.w = tf_q.getW();
    route_path_markers.markers.emplace_back(marker);
    // std::cout << "(" << path_ids_[i] << ")" << std::endl;
  }

  pub_route_path_.publish(route_path_markers);
}

//根据当前的定位信息，进行osm地图的全局规划
bool OsmRouteManager::OsmRoutePlanner(
    const cyber_msgs::LocalizationEstimate& adc_state) {
  // if (!target_id_updated_) return false;
  double current_utm_x = adc_state.pose.position.x;
  double current_utm_y = adc_state.pose.position.y;
  double current_utm_yaw = tf::getYaw(adc_state.pose.orientation);
  // int source_id =
  // osm_map_manager_->osm_map()->getNearestPointUTMXYWithHeading(
  //     current_utm_x, current_utm_y, current_utm_yaw);
  int source_id = osm_map_manager_->osm_map()->getNearestPointUTMXY(
      current_utm_x, current_utm_y);
  std::cout << "cur_x : " << current_utm_x << " ,cur_y : " << current_utm_y
            << std::endl;
  std::cout << "source id : " << source_id << std::endl;

  if (!target_id_updated_) {
    if (route_updated_ == true) {
      //找一下当前位置是否在规划的路径点中
      int path_id_index = 0;
      double min_distance = fabs(
          sqrt(pow(current_utm_x - osm_map_manager_->osm_map()
                                       ->osm_nodes()[path_ids_[path_id_index]]
                                       .utm_x,
                   2.0) +
               pow(current_utm_y - osm_map_manager_->osm_map()
                                       ->osm_nodes()[path_ids_[path_id_index]]
                                       .utm_y,
                   2.0)) -
          0.0);
      for (int i = path_id_index + 1; i < path_ids_.size(); i++) {
        double distance =
            fabs(sqrt(pow(current_utm_x - osm_map_manager_->osm_map()
                                              ->osm_nodes()[path_ids_[i]]
                                              .utm_x,
                          2.0) +
                      pow(current_utm_y - osm_map_manager_->osm_map()
                                              ->osm_nodes()[path_ids_[i]]
                                              .utm_y,
                          2.0)) -
                 0.0);
        if (distance < min_distance) {
          min_distance = distance;
          path_id_index = i;
        }
      }
      std::cout << "min_dis : " << min_distance << std::endl;
      if (min_distance < 15.0) {
        std::cout << "path_id_index : " << path_id_index << std::endl;
        std::cout << "nav id index :" << current_navigation_node_id_
                  << std::endl;
        //不需要重新route
        //更新导航点
        double delta_theta =
            NormalAngle(atan2(osm_map_manager_->osm_map()
                                      ->osm_nodes()[path_ids_[path_id_index]]
                                      .utm_y -
                                  current_utm_y,
                              osm_map_manager_->osm_map()
                                      ->osm_nodes()[path_ids_[path_id_index]]
                                      .utm_x -
                                  current_utm_x));

        if (fabs(delta_theta) < M_PI_2) {
          path_id_index = std::max(path_id_index + 2, path_id_last_index_);
          path_id_index = (path_id_index > path_ids_.size() - 1)
                              ? (path_ids_.size() - 1)
                              : path_id_index;
          current_navigation_node_id_ = path_ids_[path_id_index];
          path_id_last_index_ = path_id_index;
          // path_id_last_index_ = path_id_index;
        } else {
          //避免数组越界
          path_id_index = std::max(path_id_index + 3, path_id_last_index_);
          path_id_index = (path_id_index > path_ids_.size() - 1)
                              ? (path_ids_.size() - 1)
                              : path_id_index;
          current_navigation_node_id_ = path_ids_[path_id_index];
          path_id_last_index_ = path_id_index;
          // path_id_last_index_ = index;
        }

        //转成自车坐标系
        geometry_msgs::PoseStamped source_pt;
        if (smoothed_route_path_ && smoothed_success_) {
          source_pt.pose.position.x = smoothed_route_points_[path_id_index].x;
          source_pt.pose.position.y = smoothed_route_points_[path_id_index].y;
          source_pt.pose.position.z = 0.0;
          auto tf_q = tf::createQuaternionFromYaw(
              smoothed_route_points_[path_id_index].theta);
          source_pt.pose.orientation.x = tf_q.getX();
          source_pt.pose.orientation.y = tf_q.getY();
          source_pt.pose.orientation.z = tf_q.getZ();
          source_pt.pose.orientation.w = tf_q.getW();
        } else {
          source_pt.pose.position.x =
              osm_map_manager_->osm_map()
                  ->osm_nodes()[current_navigation_node_id_]
                  .utm_x;
          source_pt.pose.position.y =
              osm_map_manager_->osm_map()
                  ->osm_nodes()[current_navigation_node_id_]
                  .utm_y;
          source_pt.pose.position.z = 0.0;
          auto tf_q = tf::createQuaternionFromYaw(
              path_point_theta_[path_id_last_index_]);
          source_pt.pose.orientation.x = tf_q.getX();
          source_pt.pose.orientation.y = tf_q.getY();
          source_pt.pose.orientation.z = tf_q.getZ();
          source_pt.pose.orientation.w = tf_q.getW();
        }

        geometry_msgs::PoseStamped desire_pt;
        if (TransformPose(listener_, "world", "base_link", source_pt,
                          desire_pt)) {
          AERROR << "Local XY Transform Right!";
          local_navigation_pose_ = desire_pt;
        } else {
          AERROR << "Local XY Transform Wrong!";
        }
        return true;
      }
    }
  }
  osm_map_manager_->osm_map()->createNetwork();
  osm_map_manager_->osm_map()->deleteWrongDirectionEdgeOnGraph(source_id,
                                                               current_utm_yaw);
  ROS_ERROR("source ID : %d, target ID : %d", source_id, target_id_);

  std::vector<int> path_ids;
  try {
    double start_time = ros::Time::now().toSec();
    path_ids = dijkstra_planner_.findShortestPath(
        osm_map_manager_->osm_map()->getGraphOfVertex(), source_id, target_id_);
    path_found_ = true;

    ROS_WARN("OSM planner: Time of planning : %f  ms",
             1000.0 * (ros::Time::now().toSec() - start_time));
    ROS_WARN("Target ID : %d", target_id_);
  } catch (osm_parser::path_finder_algorithm::PathFinderException& e) {
    path_found_ = false;
    if (e.getErrId() ==
        osm_parser::path_finder_algorithm::PathFinderException::NO_PATH_FOUND) {
      ROS_ERROR("OSM planner : Make plan failed...");
    } else
      ROS_ERROR("OSM planner: Undefined error");
  }
  if (path_found_) {
    path_ids_.clear();
    path_ids_ = path_ids;
    GetThetaFromRoutePath();
    current_navigation_node_id_ = path_ids_[1];
    path_id_last_index_ = 1;
    route_updated_ = true;
    target_id_updated_ = false;
    if (smoothed_route_path_) {
      smoothed_success_ = SmootherRoutePath();
      std::cout << "smooth success : " << smoothed_success_ << std::endl;
      if (!smoothed_success_) smoothed_route_points_.clear();
    }
  }

  //转成自车坐标系
  // geometry_msgs::PoseStamped source_pt;
  // source_pt.pose.position.x = osm_map_manager_->osm_map()
  //                                 ->osm_nodes()[current_navigation_node_id_]
  //                                 .utm_x;
  // source_pt.pose.position.y = osm_map_manager_->osm_map()
  //                                 ->osm_nodes()[current_navigation_node_id_]
  //                                 .utm_y;
  // source_pt.pose.position.z = 0.0;
  // auto tf_q =
  //     tf::createQuaternionFromYaw(path_point_theta_[path_id_last_index_]);
  // source_pt.pose.orientation.x = tf_q.getX();
  // source_pt.pose.orientation.y = tf_q.getY();
  // source_pt.pose.orientation.z = tf_q.getZ();
  // source_pt.pose.orientation.w = tf_q.getW();
  geometry_msgs::PoseStamped source_pt;
  if (smoothed_route_path_ && smoothed_success_) {
    source_pt.pose.position.x = smoothed_route_points_[1].x;
    source_pt.pose.position.y = smoothed_route_points_[1].y;
    source_pt.pose.position.z = 0.0;
    auto tf_q = tf::createQuaternionFromYaw(smoothed_route_points_[1].theta);
    source_pt.pose.orientation.x = tf_q.getX();
    source_pt.pose.orientation.y = tf_q.getY();
    source_pt.pose.orientation.z = tf_q.getZ();
    source_pt.pose.orientation.w = tf_q.getW();
  } else {
    source_pt.pose.position.x = osm_map_manager_->osm_map()
                                    ->osm_nodes()[current_navigation_node_id_]
                                    .utm_x;
    source_pt.pose.position.y = osm_map_manager_->osm_map()
                                    ->osm_nodes()[current_navigation_node_id_]
                                    .utm_y;
    source_pt.pose.position.z = 0.0;
    auto tf_q =
        tf::createQuaternionFromYaw(path_point_theta_[path_id_last_index_]);
    source_pt.pose.orientation.x = tf_q.getX();
    source_pt.pose.orientation.y = tf_q.getY();
    source_pt.pose.orientation.z = tf_q.getZ();
    source_pt.pose.orientation.w = tf_q.getW();
  }
  geometry_msgs::PoseStamped desire_pt;
  if (TransformPose(listener_, "world", "base_link", source_pt, desire_pt)) {
    AERROR << "Local XY Transform Right!";
    local_navigation_pose_ = desire_pt;
  } else {
    AERROR << "Local XY Transform Wrong!";
  }

  return path_found_;
}

bool OsmRouteManager::OsmRoutePlanner(int source_id, int target_id) {
  int max_id = osm_map_manager_->osm_map()->osm_nodes().size() - 1;
  if (source_id > max_id) {
    source_id = max_id;
  }
  if (target_id > max_id) {
    target_id = max_id;
  }
  path_ids_.clear();
  std::vector<int> path_ids;
  try {
    double start_time = ros::Time::now().toSec();
    path_ids = dijkstra_planner_.findShortestPath(
        osm_map_manager_->osm_map()->getGraphOfVertex(), source_id, target_id);
    path_found_ = true;

    ROS_INFO("OSM planner: Time of planning : %f  ms",
             1000.0 * (ros::Time::now().toSec() - start_time));

  } catch (osm_parser::path_finder_algorithm::PathFinderException& e) {
    path_found_ = false;
    if (e.getErrId() ==
        osm_parser::path_finder_algorithm::PathFinderException::NO_PATH_FOUND) {
      ROS_ERROR("OSM planner: Make plan failed...");
    } else
      ROS_ERROR("OSM planner: Undefined error");
  }
  if (path_found_) {
    path_ids_.clear();
    path_ids_ = path_ids;
    GetThetaFromRoutePath();
    if (smoothed_route_path_) {
      smoothed_success_ = SmootherRoutePath();
      std::cout << "smooth success : " << smoothed_success_ << std::endl;
      if (!smoothed_success_) smoothed_route_points_.clear();
    }
  }
  return path_found_;
}

void OsmRouteManager::GetThetaFromRoutePath() {
  if (path_ids_.size() <= 1) return;
  path_point_theta_.clear();
  double delta_x1, delta_y1, yaw1;
  double delta_x2, delta_y2, yaw2;
  double yaw;
  delta_x1 = osm_map_manager_->osm_map()->osm_nodes()[path_ids_[1]].utm_x -
             osm_map_manager_->osm_map()->osm_nodes()[path_ids_[0]].utm_x;
  delta_y1 = osm_map_manager_->osm_map()->osm_nodes()[path_ids_[1]].utm_y -
             osm_map_manager_->osm_map()->osm_nodes()[path_ids_[0]].utm_y;
  yaw = atan2(delta_y1, delta_x1);
  path_point_theta_.emplace_back(yaw);
  for (int i = 1; i < path_ids_.size() - 1; i++) {
    delta_x1 = osm_map_manager_->osm_map()->osm_nodes()[path_ids_[i]].utm_x -
               osm_map_manager_->osm_map()->osm_nodes()[path_ids_[i - 1]].utm_x;
    delta_y1 = osm_map_manager_->osm_map()->osm_nodes()[path_ids_[i]].utm_y -
               osm_map_manager_->osm_map()->osm_nodes()[path_ids_[i - 1]].utm_y;
    delta_x2 =
        osm_map_manager_->osm_map()->osm_nodes()[path_ids_[i + 1]].utm_x -
        osm_map_manager_->osm_map()->osm_nodes()[path_ids_[i]].utm_x;
    delta_y2 =
        osm_map_manager_->osm_map()->osm_nodes()[path_ids_[i + 1]].utm_y -
        osm_map_manager_->osm_map()->osm_nodes()[path_ids_[i]].utm_y;
    yaw1 = atan2(delta_y1, delta_x1);
    yaw2 = atan2(delta_y2, delta_x2);
    yaw = NormalAngle(0.5 * (yaw1 + yaw2));
    yaw = atan2(delta_y1 + delta_y2, delta_x1 + delta_x2);
    path_point_theta_.emplace_back(yaw);
  }
  yaw = path_point_theta_.back();
  path_point_theta_.emplace_back(yaw);
}

//全局路径平滑
bool OsmRouteManager::SmootherRoutePath() {
  if (path_ids_.size() <= 1) return false;
  FemPosDeviationSmootherConfig config;
  config.max_iter = 1000;
  config.scaled_termination = true;
  config.time_limit = 0.0;
  config.verbose = false;
  config.warm_start = true;
  config.weight_fem_pos_deviation = 1.0e10;
  config.weight_path_length = 1.0;
  config.weight_ref_deviation = 1.0;
  DiscretePointsReferencelineSmoother smoother(config);

  std::vector<TrajectoryPoint> raw_points;
  for (int i = 0; i < path_ids_.size(); i++) {
    TrajectoryPoint tmp_point;
    tmp_point.x = osm_map_manager_->osm_map()->osm_nodes()[path_ids_[i]].utm_x;
    tmp_point.y = osm_map_manager_->osm_map()->osm_nodes()[path_ids_[i]].utm_y;
    raw_points.emplace_back(tmp_point);
  }

  bool smoother_success =
      smoother.Smoother(raw_points, smoothed_route_points_,
                        param_->behavior_param_.smoothed_range);
  std::cout << "Smoother success : " << smoother_success << std::endl;

  if (smoother_success) {
    //计算theta角
    double delta_x, delta_y, yaw;
    delta_x = smoothed_route_points_[1].x - smoothed_route_points_[0].x;
    delta_y = smoothed_route_points_[1].y - smoothed_route_points_[0].y;
    yaw = atan2(delta_y, delta_x);
    smoothed_route_points_[0].theta = yaw;
    for (int i = 1; i < smoothed_route_points_.size() - 1; i++) {
      delta_x =
          smoothed_route_points_[i + 1].x - smoothed_route_points_[i - 1].x;
      delta_y =
          smoothed_route_points_[i + 1].y - smoothed_route_points_[i - 1].y;
      yaw = atan2(delta_y, delta_x);
      smoothed_route_points_[i].theta = yaw;
    }
    smoothed_route_points_.back().theta =
        smoothed_route_points_[smoothed_route_points_.size() - 2].theta;
  }

  return smoother_success;
}

double OsmRouteManager::NormalAngle(double angle) {
  double tmp = (angle + M_PI) / (2 * M_PI);
  double tmp1 = (angle + M_PI) - (int)tmp * (2 * M_PI);
  if (tmp1 < 0.0) {
    tmp1 += 2 * M_PI;
  }
  return tmp1 - M_PI;
}

void OsmRouteManager::TargetPointCallback(
    const std_msgs::Int32::ConstPtr& msg) {
  target_id_updated_ = true;
  target_id_ = msg->data;
}

void OsmRouteManager::MoveBaseGoalCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  double utm_x = msg->pose.position.x;
  double utm_y = msg->pose.position.y;
  int target_id =
      osm_map_manager_->osm_map()->getNearestPointUTMXY(utm_x, utm_y);
  target_id_ = target_id;
  std::cout << "target x : " << utm_x << " ,target y : " << utm_y << std::endl;
  std::cout << "target id : " << target_id_ << std::endl;
  target_id_updated_ = true;
}

bool OsmRouteManager::TransformPoint(tf::TransformListener* listener,
                                     const std::string source_frame,
                                     const std::string desire_frame,
                                     geometry_msgs::PointStamped& source_pt,
                                     geometry_msgs::PointStamped& desire_pt) {
  source_pt.header.frame_id = source_frame;
  try {
    listener->transformPoint(desire_frame, source_pt, desire_pt);
    return true;  ///转换成功
  } catch (tf::TransformException ex) {
    // std::cout<<"Test2.2"<<std::endl;
    // ROS_ERROR("%s", ex.what());
    // ROS_ERROR("TF error1!");
    ros::Duration(1.0).sleep();
    return false;  ///转换失败
  }
}

bool OsmRouteManager::TransformPose(tf::TransformListener* listener,
                                    const std::string source_frame,
                                    const std::string desire_frame,
                                    geometry_msgs::PoseStamped& source_pose,
                                    geometry_msgs::PoseStamped& desire_pose) {
  source_pose.header.frame_id = source_frame;
  try {
    listener->transformPose(desire_frame, source_pose, desire_pose);
    return true;  ///转换成功
  } catch (tf::TransformException ex) {
    // ROS_ERROR("%s", ex.what());
    // ROS_ERROR("TF error1!");
    ros::Duration(1.0).sleep();
    return false;  ///转换失败
  }
}

void OsmRouteManager::PubSmoothedRoutePath() {
  if (!smoothed_route_path_ || !smoothed_success_) {
    PublishRoutePath();
    return;
  }
  visualization_msgs::MarkerArray route_path_markers;
  visualization_msgs::Marker marker;
  int id_count = 0;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "path_points";
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.lifetime = ros::Duration(0.2);

  for (int i = 0; i < smoothed_route_points_.size(); i++) {
    if (i == path_id_last_index_) {
      // Scale
      marker.scale.x = 5.8;
      marker.scale.y = 3.8;
      marker.scale.z = 1.8;
      // Color
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    } else {
      // Scale
      marker.scale.x = 3.8;
      marker.scale.y = 1.8;
      marker.scale.z = 1.8;
      // Color
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }

    marker.id = id_count++;
    // marker.color.a = 1.0 * (i + 1) / (path_ids_.size() + 1);
    marker.color.a = 1.0;
    marker.pose.position.x = smoothed_route_points_[i].x;
    marker.pose.position.y = smoothed_route_points_[i].y;
    marker.pose.position.z = 0.0;
    double yaw = smoothed_route_points_[i].theta;
    auto tf_q = tf::createQuaternionFromYaw(yaw);
    marker.pose.orientation.x = tf_q.getX();
    marker.pose.orientation.y = tf_q.getY();
    marker.pose.orientation.z = tf_q.getZ();
    marker.pose.orientation.w = tf_q.getW();
    route_path_markers.markers.emplace_back(marker);
    // std::cout << "(" << path_ids_[i] << ")" << std::endl;
  }

  pub_smoothed_route_rviz_.publish(route_path_markers);
}
}  // namespace planning
