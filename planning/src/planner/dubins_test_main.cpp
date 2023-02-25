#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>

#include "dubins_curve.h"

ros::Publisher pub_path_point;

visualization_msgs::MarkerArray path_markers;

void gennerator_rviz_path(const std::vector<geometry_msgs::Pose>& path_point) {
  visualization_msgs::Marker marker;
  int id = 0;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.lifetime = ros::Duration(0);
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::CYLINDER;
  for (int i = 0; i < path_point.size(); i++) {
    marker.pose.position.x = path_point[i].position.x;
    marker.pose.position.y = path_point[i].position.y;
    marker.pose.position.z = 0.0;
    marker.id = id++;
    path_markers.markers.emplace_back(marker);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "Dubins_Curve_Test");
  ros::NodeHandle nh("~");

  pub_path_point =
      nh.advertise<visualization_msgs::MarkerArray>("/pub_path_point", 5);
  double q0[3] = {0, 0, 0};
  double q1[3] = {10, 10, -1.57};

  hybrid_a_star::DubinsCurve dubins_curve;

  hybrid_a_star::DubinsPath path;

  double last_time = ros::Time::now().toSec();
  dubins_curve.dubins_shortest_path(&path, q0, q1, 6.0);

  std::cout << "dubins curve time : " << ros::Time::now().toSec() - last_time
            << std::endl;

  std::cout << "curve_type : " << path.type << std::endl;

  printf("#x,y,theta,t\n");
  dubins_curve.dubins_path_sample_many(&path, 0.1);

  std::cout << "dubins curve sample time : "
            << ros::Time::now().toSec() - last_time << std::endl;

  gennerator_rviz_path(dubins_curve.path_points());
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    pub_path_point.publish(path_markers);
    ros::spinOnce();
    loop_rate.sleep();
  }
  //   ros::spin();

  return 0;
}