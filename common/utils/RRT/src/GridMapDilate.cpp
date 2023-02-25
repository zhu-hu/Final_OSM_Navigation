//
// Created by cyber on 18-10-19.
//

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "DSTAR/dstar_map/include/create_map/map_param.h"

ros::Publisher *pub_dilate_gridmap_ = nullptr;
ros::Subscriber sub_grid_map;
cv::Mat kernel_dilate_ = cv::getStructuringElement(
    cv::MORPH_ELLIPSE, cv::Size(map_param::grid_map::kDDilateSize,
                                map_param::grid_map::kDDilateSize));

void GridMapCallback(const sensor_msgs::CompressedImageConstPtr &map_in) {
  cv::Mat local_map;
  local_map = cv::imdecode(cv::Mat(map_in->data), CV_LOAD_IMAGE_UNCHANGED);
  cv::dilate(local_map, local_map, kernel_dilate_);

  sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(std_msgs::Header(), "mono8", local_map).toImageMsg();
  pub_dilate_gridmap_->publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "GridMapDilate");
  ros::NodeHandle nh;

  ros::Publisher publisher =
      nh.advertise<sensor_msgs::Image>("/dilated_grid_map2", 1);
  pub_dilate_gridmap_ = &publisher;
  sub_grid_map =
      nh.subscribe("/perception/local_grid_map/compressed", 1, GridMapCallback);

  ros::spin();

  return 0;
}