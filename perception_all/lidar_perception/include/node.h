/*
 * @Author: Liuyaqi99 283402121@qq.com
 * @Date: 2022-09-26 11:34:46
 * @LastEditors: Liuyaqi99 283402121@qq.com
 * @LastEditTime: 2022-09-26 13:12:32
 * @FilePath: /zh_ws/src/perception/lidar_perception/include/node.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * Copyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao Tong
 * University. All rights reserved.
 * Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.02.20
 */
#ifndef NODE_H
#define NODE_H

#include <cv_bridge/cv_bridge.h>
#include <cyber_msgs/LocalizationEstimate.h>
#include <cyber_msgs/ObjectArray.h>
#include <image_transport/image_transport.h>
#include <mrpt/poses/CPose3D.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int8.h>

#include <Eigen/Dense>

#include "common/object.h"
#include "common/point_type.h"
#include "grid_map/grid_map.h"
#include "lidar_preprocess/lidar_preprocess.h"
#include "object_segmentation/object_segmenter.h"
#include "sensor_fusion/sensor_fusion.h"

class Node {
public:
  Node();

  void LivoxCloudCallback(const PointTypeCloudConstPtr &in_cloud_ptr);

  void LocalizatioinCallback(
      const cyber_msgs::LocalizationEstimate::ConstPtr &in_localization_msg);

private:
  ros::NodeHandle nh_;

  // subscriber
  ros::Subscriber sub_point_cloud_;
  ros::Subscriber sub_localization_;
  ros::Subscriber sub_mode_;

  // topic name(subscriber)
  std::string in_point_cloud_topic_;
  std::string in_localization_topic_;

  // publisher
  image_transport::Publisher pub_local_grid_map_;
  ros::Publisher pub_global_objects_;
  ros::Publisher pub_local_points_;
  ros::Publisher pub_global_points_;

  // topic name(publisher)
  std::string out_local_grid_map_topic_;
  std::string out_global_objects_topic_;

  // params input
  bool publish_local_points_ = false;
  std::string out_local_points_topic_ = "/perception/local_points";
  bool publish_global_points_ = false;
  std::string out_global_points_topic_ = "/perception/global_points";

  // frame id
  std::string local_frame_id_ = "base_link";
  std::string global_frame_id_ = "world";

  // lidar_preprocess
  LidarPreprocess::LidarPreprocessParams lp_params_;
  std::shared_ptr<LidarPreprocess> lidar_preprocess_;

  // localization
  cyber_msgs::LocalizationEstimate localization_;
  bool have_localization_ = false;

  // roi_map
  GridMap::GridMapParams gm_params_;
  std::shared_ptr<GridMap> grid_map_;
  bool pub_grid_map_flag = false;

  // object segmentation
  ObjectSegmenter::ObjectSegmenterParams os_params_;
  std::shared_ptr<ObjectSegmenter> object_segmenter_;

  // sensor fusion
  std::shared_ptr<SensorFusion> sensor_fusion_;

private:
  void PublishImage(const cv::Mat &image, const uint64_t &stamp,
                    const image_transport::Publisher &pub);

  void PublishObjects(cyber_msgs::ObjectArray &objects, const uint64_t &stamp,
                      const ros::Publisher &pub);

  void PerceptionModeCloudCallback(const std_msgs::Int8ConstPtr &msg);
};

#endif // NODE_H_
