#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

ros::Publisher pub_line_;
ros::Publisher pub_link_angle_;
double angle_zero_boas_degree_ = 0.0;

double min_distance_ = 0.0;
double max_distance_ = 0.0;
double max_fabs_angle_ = 0.0;

void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  auto laser_point = msg->ranges;
  std::cout << "laser_point_nums : " << laser_point.size() << std::endl;
  double angle_min = msg->angle_min;
  double angle_max = msg->angle_max;
  double angle_increment = msg->angle_increment;
  std::vector<cv::Point2f> scan_points_;
  for (int i = 0; i < msg->ranges.size(); i++) {
    //距离滤除
    if (msg->ranges[i] < min_distance_ || msg->ranges[i] > max_distance_ ||
        std::isnan(msg->ranges[i]))
      continue;
    double angle = angle_min + angle_increment * i;
    //角度滤除
    if (std::fabs(angle) > max_fabs_angle_) continue;
    cv::Point2f point;
    point.x = msg->ranges[i] * cos(angle);  // scale:m
    point.y = msg->ranges[i] * sin(angle);  // scale:m
    // std::cout << "x : " << point.x << "  y : " << point.y
    //           << "   range : " << msg->ranges[i] << std::endl;
    scan_points_.emplace_back(point);
  }

  std::cout << "process_point_nums : " << scan_points_.size() << std::endl;

  cv::Vec4f line_pare;

  cv::fitLine(scan_points_, line_pare, 2, 0, 1e-2, 1e-2);

  std::cout << "line pare : " << line_pare << std::endl;
  geometry_msgs::Point point0;
  point0.x = line_pare[2];
  point0.y = line_pare[3];

  double k = line_pare[1] / line_pare[0];
  double theta = 0.0;
  if (k > 0.0) {
    theta = M_PI_2 - atan2(k, 1.0);  //拖挂车向左偏为正
  } else {
    theta = -M_PI_2 - atan2(k, 1.0);  //拖挂车向右偏为负
  }
  // double theta = M_PI_2 - atan2(k, 1.0);
  std::cout << "orignal line theta : " << atan2(k, 1.0) * 180.0 / M_PI << " Deg"
            << std::endl;
  std::cout << "orignal line theta bias : "
            << atan2(k, 1.0) * 180.0 / M_PI + angle_zero_boas_degree_ << " Deg"
            << std::endl;
  std::cout << "theta : " << theta * 180.0 / M_PI << " Deg" << std::endl;
  std::cout << "theta bias : " << theta * 180.0 / M_PI - angle_zero_boas_degree_
            << " Deg" << std::endl;
  std_msgs::Float64 angle_msg;
  angle_msg.data = theta * 180.0 / M_PI - angle_zero_boas_degree_;

  pub_link_angle_.publish(angle_msg);

  geometry_msgs::Point point1, point2;
  point1.x = scan_points_[0].x;
  point1.y = k * (point1.x - point0.x) + point0.y;
  point2.x = scan_points_.back().x;
  point2.y = k * (point2.x - point0.x) + point0.y;

  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = "laser";
  line_marker.header.stamp = ros::Time::now();
  line_marker.id = 0;
  line_marker.lifetime = ros::Duration(0);
  line_marker.type = visualization_msgs::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::Marker::ADD;

  line_marker.scale.x = 0.05;
  line_marker.scale.y = 0.05;
  line_marker.scale.z = 0.05;
  line_marker.color.a = 1.0;
  line_marker.color.r = 1.0;
  line_marker.color.g = 0.0;
  line_marker.color.b = 0.0;
  geometry_msgs::Point point;
  point.x = point1.x;
  point.y = point1.y;
  line_marker.points.emplace_back(point);
  point.x = point2.x;
  point.y = point2.y;
  line_marker.points.emplace_back(point);
  pub_line_.publish(line_marker);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_test");

  ros::NodeHandle nh("~");

  ros::Subscriber sub_scan = nh.subscribe("/scan", 5, &LaserScanCallback);

  nh.param("angle_zero_bias_degree", angle_zero_boas_degree_, 0.0);
  nh.param("min_dis", min_distance_, 0.0);
  nh.param("max_dis", max_distance_, 0.0);
  nh.param("max_fabs_angle", max_fabs_angle_, 0.0);

  pub_line_ = nh.advertise<visualization_msgs::Marker>("/fitline", 5);

  pub_link_angle_ = nh.advertise<std_msgs::Float64>("/link_angle_degree", 5);

  ros::spin();

  return 0;
}