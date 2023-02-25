#ifndef LIDAR_VISUALIZE_GLOBAL_VISUALIZE_H_
#define LIDAR_VISUALIZE_GLOBAL_VISUALIZE_H_

#include <string>

#include <ros/ros.h>

#include <geometry_msgs/PolygonStamped.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

#include <cyber_msgs/LocalizationEstimate.h>
#include <cyber_msgs/ObjectArray.h>

class GlobalViewerNode {
 public:
  GlobalViewerNode();

  // Visualize object using bounding boxes and line list marker
  void ObjectArrayCallback(
      const cyber_msgs::ObjectArray::ConstPtr& object_array);

  // Subscriber localization estimation and publish tf
  void LocalizationCallback(
      cyber_msgs::LocalizationEstimate localization_estimation);

 private:
  ros::NodeHandle nh_;

  // Subscriber
  ros::Subscriber sub_object_array_;
  ros::Subscriber sub_localization_;

  // Publisher
  ros::Publisher pub_bounding_box_;
  ros::Publisher pub_orientation_;
  ros::Publisher pub_polygon_;
  ros::Publisher pub_id_and_velocity_;

  // Marker array
  visualization_msgs::MarkerArray arrow_array_;
  visualization_msgs::MarkerArray id_array_;

  // TF transform broadcaster
  tf::TransformBroadcaster* tf_br_;

  // Topic name
  std::string object_array_topic_ = "/perception/global_objects";
  std::string localization_topic_ = "/localization/estimation";
  std::string bounding_box_topic_ = "/lidar_visualize/lidar_bounding_box";
  std::string orientation_topic_ = "/lidar_visualize/lidar_orientation";
  std::string polygon_topic_ = "/lidar_visualize/lidar_polygon";
  std::string id_and_velocity_topic_ = "/lidar_visualize/lidar_text";

  // Publish params
  std::string frame_id_ = "world";
  std::string namespace_ = "lidar_perception";
};

#endif  // LIDAR_VISUALIZE_GLOBAL_VISUALIZE_H_
