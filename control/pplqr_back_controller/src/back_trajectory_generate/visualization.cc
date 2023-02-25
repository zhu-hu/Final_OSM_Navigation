#include "visualization.h"

namespace visualization {

double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

// Publish whole trajectoties based on planning result
void PublishGlobalTrajectory(
    const ros::Publisher &pub,
    const std::list<cyber_msgs::LocalTrajList> &trajectories) {
  if (trajectories.size() == 0) {
    AWARN << "No trajectories to visualize!";
    return;
  };
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "world";
  for (auto &traj : trajectories) {
    for (auto &point : traj.points) {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header = path_msg.header;
      pose_msg.pose.position = point.position;
      pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(point.theta);
      path_msg.poses.emplace_back(pose_msg);
    }
  }
  pub.publish(path_msg);
}

void PublishRealPath(const ros::Publisher &pub, const nav_msgs::Path &real_path,
                     const std::string color) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker path_marker;
  int id = 0;
  path_marker.header.frame_id = "world";
  path_marker.header.stamp = ros::Time::now();
  path_marker.action = visualization_msgs::Marker::ADD;
  path_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_marker.color.r = 0.0f;
  path_marker.color.g = 1.0f;
  path_marker.color.b = 0.0f;
  if (color == "red") {
    path_marker.color.r = 1.0f;
    path_marker.color.g = 0.0f;
    path_marker.color.b = 0.0f;
  }
  if (color == "blue") {
    path_marker.color.r = 0.0f;
    path_marker.color.g = 0.0f;
    path_marker.color.b = 1.0f;
  }

  if (color == "yellow") {
    path_marker.color.r = 1.0f;
    path_marker.color.g = 1.0f;
    path_marker.color.b = 0.0f;
  }
  path_marker.color.a = 1.0f;
  path_marker.scale.x = 0.1;
  path_marker.scale.y = 0.1;
  path_marker.scale.z = 0.1;
  path_marker.lifetime = ros::Duration(0.0);
  path_marker.id = id++;

  for (const auto &path_pose : real_path.poses) {
    geometry_msgs::Point point;
    point.x = path_pose.pose.position.x;
    point.y = path_pose.pose.position.y;
    point.z = 0.0;
    path_marker.points.emplace_back(point);
  }

  marker_array.markers.emplace_back(path_marker);

  pub.publish(marker_array);
}

void PubLocalMap(const ros::Publisher &pub, const cv::Mat &map_in,
                 const double &originX, const double &originY) {
  if (map_in.rows == 0 || map_in.cols == 0) {
    ROS_ERROR("NO MAP IN!!");
    return;
  }

  int newHeight = map_in.rows;
  int newWidth = map_in.cols;

  nav_msgs::OccupancyGrid::Ptr grid;
  grid.reset(new nav_msgs::OccupancyGrid);
  grid->header.frame_id = "world";
  grid->header.stamp = ros::Time::now();
  grid->info.height = newHeight;
  grid->info.width = newWidth;
  grid->info.resolution = 0.1;
  grid->info.origin.position.x = originX;
  grid->info.origin.position.y = originY;
  grid->info.origin.orientation.y = 0.0f;
  grid->info.origin.orientation.w = 0.0f;
  // grid->header.stamp = header_.stamp;

  for (int row = map_in.rows - 1; row >= 0; row--) {
    for (int col = 0; col < map_in.cols; col++) {
      if (map_in.at<uchar>(row, col) == 255)
        grid->data.emplace_back(100);  ///障碍物
      else if (map_in.at<uchar>(row, col) == 0)
        grid->data.emplace_back(0);  ///无障碍物
      else {
        grid->data.emplace_back(50);  ///未知
        // std::cout<<int(map_in.at<uchar>(row, col))<<std::endl;
      }
    }
  }

  pub.publish(*grid);
}

void ShowVehiclePose(const ros::Publisher &pub,
                     const cyber_msgs::LocalizationEstimateConstPtr &pose_in) {
  visualization_msgs::MarkerArray pos_marker;
  visualization_msgs::Marker cube_marker;
  cube_marker.header = pose_in->header;
  cube_marker.header.stamp = ros::Time::now();
  cube_marker.action = visualization_msgs::Marker::ADD;
  cube_marker.type = visualization_msgs::Marker::CUBE;
  cube_marker.pose = pose_in->pose;
  double shift_distance =
      vehicle_param::kLength / 2 - vehicle_param::kRearToBack;
  cube_marker.pose.position.x +=
      std::cos(tf::getYaw(pose_in->pose.orientation)) * shift_distance;
  cube_marker.pose.position.y +=
      std::sin(tf::getYaw(pose_in->pose.orientation)) * shift_distance;
  cube_marker.color.r = 1.0f;
  cube_marker.color.g = 0.0f;
  cube_marker.color.b = 0.0f;
  cube_marker.color.a = 0.3f;
  cube_marker.scale.x = vehicle_param::kLength;
  cube_marker.scale.y = vehicle_param::kWidth;
  cube_marker.scale.z = 0.1;
  cube_marker.lifetime = ros::Duration(0.0);
  cube_marker.id = 0;
  // cube_marker.frame_locked = true;
  pos_marker.markers.emplace_back(cube_marker);
  visualization_msgs::Marker center_marker;
  center_marker.header = pose_in->header;
  center_marker.header.stamp = ros::Time::now();
  center_marker.action = visualization_msgs::Marker::ADD;
  center_marker.type = visualization_msgs::Marker::CYLINDER;
  center_marker.pose = pose_in->pose;
  center_marker.color.r = 1.0f;
  center_marker.color.g = 1.0f;
  center_marker.color.b = 0.0f;
  center_marker.color.a = 1.0f;
  center_marker.scale.x = 0.1;
  center_marker.scale.y = 0.1;
  center_marker.scale.z = 0.2;
  center_marker.lifetime = ros::Duration(0.0);
  center_marker.id = 1;
  // center_marker.frame_locked = true;
  pos_marker.markers.emplace_back(center_marker);

  pub.publish(pos_marker);
}

void ShowVehiclePose(const ros::Publisher &pub,
                     const cyber_msgs::LocalizationEstimateConstPtr &pose_in,
                     const cyber_msgs::LinkAngleConstPtr &link_angle_in) {
  //连接处的位姿
  geometry_msgs::Pose link_point_pose;
  link_point_pose = pose_in->pose;
  link_point_pose.position.x =
      pose_in->pose.position.x -
      vehicle_param::M1 * cos(tf::getYaw(pose_in->pose.orientation));
  link_point_pose.position.y =
      pose_in->pose.position.y -
      vehicle_param::M1 * sin(tf::getYaw(pose_in->pose.orientation));

  double beta = link_angle_in->link_angle_degree * M_PI / 180.0;
  double theta2 = NormalizeAngle(tf::getYaw(pose_in->pose.orientation) - beta);

  //拖挂车车体几何中心位姿
  geometry_msgs::Pose trailer_center_pose;

  auto tf_q = tf::createQuaternionFromYaw(theta2);

  trailer_center_pose.orientation.x = tf_q.getX();
  trailer_center_pose.orientation.y = tf_q.getY();
  trailer_center_pose.orientation.z = tf_q.getZ();
  trailer_center_pose.orientation.w = tf_q.getW();

  trailer_center_pose.position.x =
      link_point_pose.position.x -
      (vehicle_param::ForwardLength + vehicle_param::Length2 / 2.0) *
          cos(theta2);
  trailer_center_pose.position.y =
      link_point_pose.position.y -
      (vehicle_param::ForwardLength + vehicle_param::Length2 / 2.0) *
          sin(theta2);
  trailer_center_pose.position.z = 0.0;

  //拖挂车后轴中心位姿
  geometry_msgs::Pose trailer_back_center_pose = link_point_pose;
  trailer_back_center_pose.position.x =
      link_point_pose.position.x - vehicle_param::L2 * cos(theta2);
  trailer_back_center_pose.position.y =
      link_point_pose.position.y - vehicle_param::L2 * sin(theta2);

  /***************************************
   * 可视化
   * ************************************/
  visualization_msgs::MarkerArray pos_marker;
  int id = 0;
  //可视化牵引车车体
  visualization_msgs::Marker cube_marker;
  cube_marker.header = pose_in->header;
  cube_marker.header.stamp = ros::Time::now();
  cube_marker.action = visualization_msgs::Marker::ADD;
  cube_marker.type = visualization_msgs::Marker::CUBE;
  cube_marker.pose = pose_in->pose;
  double shift_distance =
      vehicle_param::kLength / 2 - vehicle_param::kRearToBack;
  cube_marker.pose.position.x +=
      std::cos(tf::getYaw(pose_in->pose.orientation)) * shift_distance;
  cube_marker.pose.position.y +=
      std::sin(tf::getYaw(pose_in->pose.orientation)) * shift_distance;
  cube_marker.color.r = 1.0f;
  cube_marker.color.g = 0.0f;
  cube_marker.color.b = 0.0f;
  cube_marker.color.a = 0.5f;
  cube_marker.scale.x = vehicle_param::kLength;
  cube_marker.scale.y = vehicle_param::kWidth;
  cube_marker.scale.z = 0.1;
  cube_marker.lifetime = ros::Duration(0.0);
  cube_marker.id = id++;
  // cube_marker.frame_locked = true;
  pos_marker.markers.emplace_back(cube_marker);

  //可视化拖挂车车体
  cube_marker.id = id++;
  cube_marker.pose = trailer_center_pose;
  cube_marker.scale.x = vehicle_param::Length2;
  cube_marker.scale.y = vehicle_param::W2;
  cube_marker.scale.z = 0.1;
  pos_marker.markers.emplace_back(cube_marker);

  //可视化牵引车后轴中心
  visualization_msgs::Marker center_marker;
  center_marker.header = pose_in->header;
  center_marker.header.stamp = ros::Time::now();
  center_marker.action = visualization_msgs::Marker::ADD;
  center_marker.type = visualization_msgs::Marker::CYLINDER;
  center_marker.pose = pose_in->pose;
  center_marker.color.r = 1.0f;
  center_marker.color.g = 1.0f;
  center_marker.color.b = 0.0f;
  center_marker.color.a = 1.0f;
  center_marker.scale.x = 0.1;
  center_marker.scale.y = 0.1;
  center_marker.scale.z = 0.2;
  center_marker.lifetime = ros::Duration(0.0);
  center_marker.id = id++;
  // center_marker.frame_locked = true;
  pos_marker.markers.emplace_back(center_marker);

  //可视化连接点
  center_marker.id = id++;
  center_marker.pose = link_point_pose;
  pos_marker.markers.emplace_back(center_marker);

  //可视化拖挂车后轴中心
  center_marker.id = id++;
  center_marker.pose = trailer_back_center_pose;
  pos_marker.markers.emplace_back(center_marker);

  //可视化牵引车和连接轴的连杆
  visualization_msgs::Marker line_marker;
  line_marker.header = pose_in->header;
  line_marker.header.stamp = ros::Time::now();
  line_marker.color.g = 0.0;
  line_marker.color.b = 1.0;
  line_marker.color.r = 0.0;
  line_marker.color.a = 1.0;
  line_marker.id = id++;
  line_marker.type = visualization_msgs::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::Marker::ADD;
  line_marker.scale.x = 0.1;
  line_marker.scale.y = 0.1;
  line_marker.scale.z = 0.1;
  geometry_msgs::Point tmp;
  tmp.x = pose_in->pose.position.x -
          vehicle_param::K1 * cos(tf::getYaw(pose_in->pose.orientation));
  tmp.y = pose_in->pose.position.y -
          vehicle_param::K1 * sin(tf::getYaw(pose_in->pose.orientation));
  tmp.z = 0.0;
  line_marker.points.emplace_back(tmp);
  tmp.x = link_point_pose.position.x;
  tmp.y = link_point_pose.position.y;
  tmp.z = 0.0;
  line_marker.points.emplace_back(tmp);
  line_marker.lifetime = ros::Duration(0);
  pos_marker.markers.emplace_back(line_marker);

  //可视化连接轴与拖挂车的连杆
  line_marker.id = id++;
  line_marker.points.clear();
  tmp.x = link_point_pose.position.x;
  tmp.y = link_point_pose.position.y;
  tmp.z = 0.0;
  line_marker.points.emplace_back(tmp);
  tmp.x =
      link_point_pose.position.x - vehicle_param::ForwardLength * cos(theta2);
  tmp.y =
      link_point_pose.position.y - vehicle_param::ForwardLength * sin(theta2);
  tmp.z = 0.0;
  line_marker.points.emplace_back(tmp);
  pos_marker.markers.emplace_back(line_marker);

  //添加Tractor和Trailer的文字显示
  visualization_msgs::Marker text_marker;
  text_marker.header = pose_in->header;
  text_marker.header.stamp = ros::Time::now();
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.r = 1.0;
  text_marker.color.a = 1.0;
  text_marker.id = id++;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.lifetime = ros::Duration(0);
  text_marker.text = std::string("Tractor");
  text_marker.pose = pose_in->pose;
  text_marker.pose.position.x +=
      std::cos(tf::getYaw(pose_in->pose.orientation)) * shift_distance;
  text_marker.pose.position.y +=
      std::sin(tf::getYaw(pose_in->pose.orientation)) * shift_distance;
  text_marker.scale.x = 0.5;
  text_marker.scale.y = 0.5;
  text_marker.scale.z = 0.6;
  pos_marker.markers.emplace_back(text_marker);

  text_marker.id = id++;
  text_marker.text = std::string("Trailer");
  text_marker.pose = trailer_center_pose;
  pos_marker.markers.emplace_back(text_marker);

  pub.publish(pos_marker);
}

}  // namespace visualization
