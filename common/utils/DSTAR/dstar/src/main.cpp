#include "Dstar/Dstar.h"
#include <memory>
#include <image_transport/image_transport.h>
#include "cyber_msgs/LocalizationEstimate.h"

Dstar* dstar = nullptr;
ros::NodeHandle* P_nh = nullptr;
geometry_msgs::PoseStamped cp;
ros::Publisher* pub_path = nullptr;
ros::Publisher* pub_target = nullptr;

bool TaskCallback(dstar_map::MapUpdate::Request& req, dstar_map::MapUpdate::Response &res) {
  ROS_INFO("Get task!!!");
  if(req.init){
    if(dstar != nullptr)
      free(dstar);
    std::cout<<"Construct Dstar Class!!"<<std::endl;
    dstar = new Dstar(P_nh, 127, 1.0, pub_path, pub_target);
    dstar->InitMap(cp, req.goal_x, req.goal_y, req.goal_theta, req.map_length, req.data);
    cyber_msgs::LocalTrajList path;
    if(!dstar->RePlan(cp, &path, 10.0)){
      ROS_WARN("No Path Init!!!");
      return true;
    };
    dstar->PubLocalTarget(cp, &path);
    dstar->PubPath(&path);
  }
  else{
    ROS_INFO("Updating Map!!!");
    dstar->UpdateMap(req.data);
    cyber_msgs::LocalTrajList path;
    if(!dstar->RePlan(cp, &path)){
      ROS_WARN("No Path!!!");
      return true;
    };
    dstar->PubLocalTarget(cp, &path);
    dstar->PubPath(&path);
  }
  return true;
}

void CurrentPoseCallback(const cyber_msgs::LocalizationEstimate& msg) {
  cp.header.stamp = msg.header.stamp;
  cp.header.frame_id = "world";
  cp.pose.position.x = msg.pose.position.x;
  cp.pose.position.y = msg.pose.position.y;
  cp.pose.position.z = msg.pose.position.z;
  cp.pose.orientation = msg.pose.orientation;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "DStar");
  ros::NodeHandle pnh("~");
  P_nh = &pnh;
  ros::ServiceServer task_srv = pnh.advertiseService("/DSMap/task_service", &TaskCallback);
  ros::Subscriber sub_cp = pnh.subscribe("/localization/estimation", 1, &CurrentPoseCallback);
  ros::Publisher path = pnh.advertise<nav_msgs::Path>("/DStar/dstar_path_rviz",1);
  ros::Publisher target = pnh.advertise<geometry_msgs::PoseStamped>("/DStar/local_target", 1);
  pub_path = &path;
  pub_target = &target;

  ros::spin();

  return 0;
}

