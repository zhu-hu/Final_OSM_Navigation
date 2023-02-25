#include "manager/publisher_manager.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace planning {
PublisherManager::PublisherManager(ros::NodeHandle *nh) {
  nh_ = nh;
  pub_trajectory_ =
      nh_->advertise<cyber_msgs::LocalTrajList>("/local_trajectory", 10);
  pub_state_ = nh_->advertise<visualization_msgs::Marker>("/planning/stage", 1);
  pub_traffic_ =
      nh_->advertise<visualization_msgs::Marker>("/planning/traffic", 1);
  pub_stager_mode_ = nh_->advertise<std_msgs::Int8>("/stager_mode", 1);
  pub_stager_distance_ =
      nh_->advertise<std_msgs::Float64>("/stager_distance", 1);
  pub_text_ = nh_->advertise<visualization_msgs::Marker>("/planning/text", 1);
  pub_point_ = nh_->advertise<visualization_msgs::Marker>("/planning/point", 1);
  pub_trajectories_visualization_ =
      nh_->advertise<visualization_msgs::MarkerArray>(
          "/visualization/trajectories", 10);
  pub_dis_to_station_ =
      nh_->advertise<std_msgs::Float64>("/planning/station_distance", 1);
  pub_station_status_ =
      nh_->advertise<std_msgs::Int32>("/planning/station_status", 1);
  pub_parking_trigger_ = nh_->advertise<std_msgs::Bool>("/parking/trigger", 10);
  pub_routing_ =
      nh_->advertise<visualization_msgs::MarkerArray>("route_marker", 1);

  pub_rrt_path_ = nh_->advertise<nav_msgs::Path>("/planning/rrt_path", 5);
  pub_local_point_ =
      nh_->advertise<visualization_msgs::Marker>("/planning/local_point", 5);
  pub_local_target_ =
      nh_->advertise<visualization_msgs::Marker>("/planning/local_target", 5);

  pub_visualization_trajectories_ =
      nh_->advertise<visualization_msgs::MarkerArray>(
          "/planning/visualization/trajectories", 10);
}

PublisherManager::~PublisherManager() {}

void PublisherManager::ShowTrajectories(
    const std::vector<ReferenceLine> &lines) {
  int id = 0;
  visualization_msgs::MarkerArray trajectories_visualization;
  for (const auto &line : lines) {
    for (const auto &derived_trajectory : line.derived_trajectories()) {
      for (size_t i = 0; i < derived_trajectory.points().size() - 1; i++) {
        visualization_msgs::Marker line;
        line.header.stamp = ros::Time::now();
        line.header.frame_id = "world";
        line.action = visualization_msgs::Marker::ADD;
        line.type = visualization_msgs::Marker::LINE_LIST;
        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 0.7;
        line.color = color;
        line.scale.x = 0.1;
        line.scale.y = 0.1;
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
      // z += 1.0;
      visualization_msgs::Marker cost_vis;
      cost_vis.id = id++;
      cost_vis.header.stamp = ros::Time::now();
      cost_vis.header.frame_id = "world";
      cost_vis.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      cost_vis.action = visualization_msgs::Marker::ADD;
      cost_vis.scale.x = 1;
      cost_vis.scale.y = 1;
      cost_vis.scale.z = 1;
      cost_vis.pose.position.x = derived_trajectory.points().back().x;
      cost_vis.pose.position.y = derived_trajectory.points().back().y;
      cost_vis.pose.position.z = 0.0;
      cost_vis.pose.orientation.x = 0.0;
      cost_vis.pose.orientation.y = 0.0;
      cost_vis.pose.orientation.z = 0.0;
      cost_vis.pose.orientation.w = 1.0;
      cost_vis.scale.z = 1;
      cost_vis.color.r = 1.0f;
      cost_vis.color.g = 0.0f;
      cost_vis.color.b = 1.0f;
      cost_vis.color.a = 1.0;
      int cost = derived_trajectory.GetCost();
      std::string text = std::to_string(cost);
      cost_vis.text = text;
      cost_vis.lifetime = ros::Duration(0.2);
      trajectories_visualization.markers.emplace_back(cost_vis);
    }
  }
  pub_trajectories_visualization_.publish(trajectories_visualization);
}

void PublisherManager::PublishTrajectory(Trajectory *const best_trajectory) {
  cyber_msgs::LocalTrajList control_trajectory;
  if (nullptr == best_trajectory) {
    AERROR << "No best trajectory to publish!";
    //如果没有规划的轨迹，发一个点到控制端，并且这个点的mode是32，方便控制端对速度进行特殊处理
    cyber_msgs::LocalTrajPoint point;
    point.mode = 32;
    control_trajectory.points.emplace_back(point);
    pub_trajectory_.publish(control_trajectory);
    return;
  }
  for (auto raw_p : best_trajectory->points()) {
    cyber_msgs::LocalTrajPoint tmp_point;
    tmp_point.position.x = raw_p.x;
    tmp_point.position.y = raw_p.y;
    tmp_point.velocity = raw_p.velocity;
    tmp_point.kappa = raw_p.kappa;
    tmp_point.theta = raw_p.theta;
    tmp_point.s = raw_p.s;
    control_trajectory.points.emplace_back(tmp_point);
  }
  pub_trajectory_.publish(control_trajectory);
}

void PublisherManager::PublishStage(const std::string stage) {
  visualization_msgs::Marker marker;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = -2.0;
  marker.lifetime = ros::Duration(0.15);
  marker.text = stage;
  marker.color.a = 1.0;
  marker.color.b = 0.9;
  marker.color.g = 0.9;
  marker.color.r = 0.9;
  marker.scale.z = 1.5;
  pub_state_.publish(marker);
}

void PublisherManager::PublishTrafficLight(int traffic_light_type_) {
  // 0为红灯，1为绿灯，2为未响应,3为Stop Sign
  visualization_msgs::Marker marker;
  marker.id = 1;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.pose.position.x = -2.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 4.0;
  marker.lifetime = ros::Duration(0.15);
  if (traffic_light_type_ == 0) {
    marker.text = "Red";
    marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker.color.g = 0.0;
    marker.color.r = 1.0;
    marker.scale.z = 1.0;
  } else if (traffic_light_type_ == 1) {
    marker.text = "Green";
    marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker.color.g = 1.0;
    marker.color.r = 0.0;
    marker.scale.z = 1.0;
  } else if (traffic_light_type_ == 3) {
    marker.text = "Stop Sign";
    marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker.color.g = 0.0;
    marker.color.r = 1.0;
    marker.scale.z = 1.0;
  } else {
    marker.text = "No response!";
    marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker.color.g = 0.0;
    marker.color.r = 0.0;
    marker.scale.z = 1.0;
  }
  pub_traffic_.publish(marker);
}

void PublisherManager::PublishWaitTime(double time_, double pass_time_) {
  visualization_msgs::Marker marker;
  marker.id = 2;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.pose.position.x = -2.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 3.0;
  marker.lifetime = ros::Duration(0.15);
  std::string time = std::to_string(time_);
  std::string pass_time = std::to_string(pass_time_);
  std::string text;
  text = " Already waiting for " + pass_time + " s" + " / " + time;
  marker.text = text;
  marker.color.a = 1.0;
  marker.color.b = 0.2;
  marker.color.g = 0.3;
  marker.color.r = 1.0;
  marker.scale.z = 1.0;
  pub_traffic_.publish(marker);
}

void PublisherManager::PublishPlanTime(double time_, double pass_time_,
                                       std::string able, std::string planner) {
  visualization_msgs::Marker marker;
  marker.id = 3;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.pose.position.x = -2.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 3.0;
  marker.lifetime = ros::Duration(0.15);
  std::string time = std::to_string(time_);
  std::string pass_time = std::to_string(pass_time_);
  std::string text;
  text =
      planner + " " + able + " to plan for " + pass_time + "s" + " / " + time;
  marker.text = text;
  if (able == "able") {
    marker.color.a = 1.0;
    marker.color.b = 0.3;
    marker.color.g = 1.0;
    marker.color.r = 0.3;
  } else {
    marker.color.a = 1.0;
    marker.color.b = 0.2;
    marker.color.g = 0.3;
    marker.color.r = 1.0;
  }
  marker.scale.z = 1.0;
  pub_traffic_.publish(marker);
}

void PublisherManager::PublishText(const std::string &text) {
  visualization_msgs::Marker marker;
  marker.id = 5;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.pose.position.x = -2.0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 5.0;
  marker.lifetime = ros::Duration(0.15);
  marker.text = text;
  marker.color.a = 1.0;
  marker.color.b = 0.1;
  marker.color.g = 0.1;
  marker.color.r = 1.0;
  marker.scale.z = 5.0;
  pub_text_.publish(marker);
}

void PublisherManager::PublishText(const std::string &text, int color_type) {
  visualization_msgs::Marker marker;
  marker.id = 6;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0;
  marker.pose.position.z = -4.0;
  marker.lifetime = ros::Duration(0.15);
  marker.text = text;
  if (color_type == 0) {
    marker.color.a = 1.0;
    marker.color.b = 0.1;
    marker.color.g = 1;
    marker.color.r = 0.1;
  } else {
    marker.color.a = 1.0;
    marker.color.b = 0.1;
    marker.color.g = 0.1;
    marker.color.r = 1.0;
  }
  marker.scale.z = 2.0;
  pub_text_.publish(marker);
}

void PublisherManager::PublishPoint(const TrajectoryPoint &trajectorypoint) {
  visualization_msgs::Marker marker;
  marker.id = 6;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.ns = "dstar_goal";
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.pose.position.x = trajectorypoint.x;
  marker.pose.position.y = trajectorypoint.y;
  marker.pose.position.z = 0;
  marker.lifetime = ros::Duration(0);
  marker.color.a = 1.0;
  marker.color.b = 0.3;
  marker.color.g = 0.3;
  marker.color.r = 1.0;
  marker.scale.x = 2;
  marker.scale.y = 2;
  marker.scale.z = 2;
  pub_point_.publish(marker);
}
void PublisherManager::PublishStagerMode(const StagerMode stager_mode,
                                         const uint32_t &emergency_mode_) {
  std_msgs::Int8 stager_int;
  if (emergency_mode_ == 0) {
    if (stager_mode == STAGERNORMAL)
      stager_int.data = 0;
    else if (stager_mode == STAGERSTRAIGHT)
      stager_int.data = 1;
    else if (stager_mode == STAGERLEFTTURN)
      stager_int.data = 2;
    else if (stager_mode == STAGERRIGHTTURN)
      stager_int.data = 3;
    else if (stager_mode == STAGERUTURN)
      stager_int.data = 4;
    else if (stager_mode == STAGERCOUNTRYROAD)
      stager_int.data = 5;
    else if (stager_mode == STAGERROADCHECK)
      stager_int.data = 6;
    else if (stager_mode == STAGERPAKING)
      stager_int.data = 7;
    else if (stager_mode == STAGERSTART)
      stager_int.data = 8;
    else if (stager_mode == STAGERLANECHANGE)
      stager_int.data = 9;
  } else if (emergency_mode_ == 2)  // STAGEREMERGENCYSTOP
    stager_int.data = 20;
  else if (emergency_mode_ == 1)  // STAGEREMERGENCYAVOID
    stager_int.data = 21;
  pub_stager_mode_.publish(stager_int);
}

void PublisherManager::PublishVisualizationTrajectories(
    const std::vector<Trajectory> &trajectories) const {
  int id = 0;
  visualization_msgs::MarkerArray trajectories_visualization;

  for (const auto &derived_trajectory : trajectories) {
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
      line.scale.x = 1.0;
      line.scale.y = 1.0;
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
  pub_visualization_trajectories_.publish(trajectories_visualization);
}

void PublisherManager::PublishStagerDistance(double stager_distance) {
  std_msgs::Float64 stager_distance_msg;
  stager_distance_msg.data = stager_distance;
  pub_stager_distance_.publish(stager_distance_msg);
}

void PublisherManager::PublishDisToStation(double distance) {
  std_msgs::Float64 distance_to_station_msg;
  distance_to_station_msg.data = distance;
  pub_dis_to_station_.publish(distance_to_station_msg);
}

void PublisherManager::PublishStationStatus(int station_status) {
  std_msgs::Int32 station_status_msg;
  station_status_msg.data = station_status;
  pub_station_status_.publish(station_status_msg);
}

void PublisherManager::PublishParkingTrigger(bool parking_trigger) {
  std_msgs::Bool parking_trigger_msg;
  parking_trigger_msg.data = parking_trigger;
  pub_parking_trigger_.publish(parking_trigger_msg);
}

void PublisherManager::PublishDriveMode(bool is_auto_drive) {
  visualization_msgs::Marker marker;
  marker.id = 61;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 3.0;
  marker.lifetime = ros::Duration(0.15);
  if (is_auto_drive) {
    marker.text = "Auto";
    marker.color.a = 0.5;
    marker.color.b = 0.1;
    marker.color.g = 1;
    marker.color.r = 0.1;
  } else {
    marker.text = "Manual";
    marker.color.a = 0.5;
    marker.color.b = 0.1;
    marker.color.g = 0.1;
    marker.color.r = 1.0;
  }
  marker.scale.z = 1.0;
  pub_text_.publish(marker);
}

void PublisherManager::PublishRoutingResult(
    const std::vector<cyber_msgs::LaneWaypoint> &routing_result_list) {
  visualization_msgs::MarkerArray markers_for_publish;
  int marker_count = 0;
  for (auto lane : routing_result_list) {
    visualization_msgs::Marker marker;
    // Set header
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "routing_result";
    marker.id = marker_count;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    // Set position
    marker.pose.position.x = lane.pose.x;
    marker.pose.position.y = lane.pose.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1.0;
    // Set scale
    marker.scale.z = 3;
    // Set color
    marker.color.r = 0.2f;
    marker.color.g = 0.9f;
    marker.color.b = 0.2f;
    marker.color.a = 1.0;
    // Set text
    marker.text = std::to_string(marker_count) + ":" + lane.id;
    marker.lifetime = ros::Duration(0.15);
    ++marker_count;
    markers_for_publish.markers.push_back(marker);
  }
  // ROS_INFO("Generated LaneWaypoints markers!");
  pub_routing_.publish(markers_for_publish);
}

void PublisherManager::PublishRRTPath(const nav_msgs::Path &path) {
  pub_rrt_path_.publish(path);
}

void PublisherManager::PublishLocalPoint(const TrajectoryPoint &point,
                                         const std::string &color) {
  visualization_msgs::Marker marker;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = 0;
  auto tf_q = tf::createQuaternionFromYaw(point.theta);
  marker.pose.orientation.x = tf_q.getX();
  marker.pose.orientation.y = tf_q.getY();
  marker.pose.orientation.z = tf_q.getZ();
  marker.pose.orientation.w = tf_q.getW();
  marker.lifetime = ros::Duration(0.2);
  if (color == "yellow") {
    marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker.color.g = 1.0;
    marker.color.r = 1.0;
  } else {
    marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker.color.g = 1.0;
    marker.color.r = 0.0;
  }
  marker.scale.x = 4.0;
  marker.scale.y = 2.0;
  marker.scale.z = 2.0;
  pub_local_point_.publish(marker);
}

void PublisherManager::PublishLocalTarget(const TrajectoryPoint &point) {
  visualization_msgs::Marker marker;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  marker.pose.position.z = 0;
  auto tf_q = tf::createQuaternionFromYaw(point.theta);
  marker.pose.orientation.x = tf_q.getX();
  marker.pose.orientation.y = tf_q.getY();
  marker.pose.orientation.z = tf_q.getZ();
  marker.pose.orientation.w = tf_q.getW();
  marker.lifetime = ros::Duration(0.2);

  marker.color.a = 1.0;
  marker.color.b = 0.0;
  marker.color.g = 1.0;
  marker.color.r = 1.0;

  marker.scale.x = 4.0;
  marker.scale.y = 2.0;
  marker.scale.z = 2.0;
  pub_local_target_.publish(marker);
}
}  // namespace planning
