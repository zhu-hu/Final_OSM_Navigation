#ifndef STATEMACHINE_GRID_MAP_EMANAGER_H
#define STATEMACHINE_GRID_MAP_EMANAGER_H

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

#include "opencv/cv.h"
#include "parameter/tiggo_model.h"
#include "sensor_msgs/Image.h"

namespace planning {

class GridMapManager {
 public:
  GridMapManager(ros::NodeHandle *nh, Parameter *param);
  ~GridMapManager();

  inline const cv::Mat grid_map() { return grid_map_; }
  inline const cv::Mat dilate_grid_map() { return dilate_grid_map_; }

  inline const cv::Mat fs_grid_map() { return fs_grid_map_; }
  inline const cv::Mat dilate_fs_grid_map() { return dilate_fs_grid_map_; }

  void GridMapCallback(const sensor_msgs::CompressedImageConstPtr &map_in);

  void FsGridMapCallback(
      const sensor_msgs::CompressedImageConstPtr &fs_grid_map_in);

 private:
  ros::NodeHandle *nh_;
  ros::Subscriber sub_grid_map_;
  //发送膨胀后的gridmap
  ros::Publisher pub_dilate_grid_map_;
  //感知发送的局部栅格图,转成灰度图
  cv::Mat grid_map_;
  //感知发送的局部栅格图,经过膨胀之后的图
  cv::Mat dilate_grid_map_;
  Parameter *param_;

  //订阅感知发过来的free_space的栅格图
  ros::Subscriber sub_fs_grid_map_;
  ros::Publisher pub_fs_grid_map_;
  cv::Mat fs_grid_map_;
  cv::Mat dilate_fs_grid_map_;
};
}  // namespace planning

#endif