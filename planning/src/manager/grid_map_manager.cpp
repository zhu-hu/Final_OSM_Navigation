#include "manager/grid_map_manager.h"

namespace planning {
GridMapManager::GridMapManager(ros::NodeHandle *nh, Parameter *param)
    : nh_(nh), param_(param) {
  sub_grid_map_ = nh_->subscribe("/perception/local_grid_map/compressed", 2,
                                 &GridMapManager::GridMapCallback, this);
  sub_fs_grid_map_ =
      nh_->subscribe("/perception/freespace_grid_map/compressed", 5,
                     &GridMapManager::FsGridMapCallback, this);
  pub_dilate_grid_map_ =
      nh_->advertise<sensor_msgs::Image>("/dilated_grid_map", 5, this);
  pub_fs_grid_map_ =
      nh_->advertise<sensor_msgs::Image>("/fs_grid_map", 5, this);
}

GridMapManager::~GridMapManager() {}

void GridMapManager::GridMapCallback(
    const sensor_msgs::CompressedImageConstPtr &map_in) {
  if (!map_in->data.empty()) {
    grid_map_ = cv::imdecode(cv::Mat(map_in->data), CV_LOAD_IMAGE_UNCHANGED);
    cv::threshold(grid_map_, grid_map_, 63, 255, CV_THRESH_BINARY);
    // const double grid_map_pixel_scale = param_->grid_map_param_.pixel_scale;
    // const double dilate_distance_of_static_obstacles =
    // param_->behavior_param_.lat_safe_distance_static_obstacle; const int
    // dilate_size = 2 * dilate_distance_of_static_obstacles *
    // grid_map_pixel_scale;

    // cv::Mat kernel_dilate = cv::getStructuringElement(cv::MORPH_RECT,
    // cv::Size(dilate_size, dilate_size)); cv::dilate(grid_map_, grid_map_,
    // kernel_dilate);
    cv::Mat kernel_dilate =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::Mat local_map;
    // cv_bridge::toCvShare(map_in, "mono8")->image.copyTo(local_map);
    cv_bridge::toCvCopy(map_in, "mono8")->image.copyTo(local_map);
    cv::dilate(local_map, dilate_grid_map_, kernel_dilate);

    //将膨胀之后的图像发送出去
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "mono8", dilate_grid_map_)
            .toImageMsg();
    pub_dilate_grid_map_.publish(msg);
  }
}

void GridMapManager::FsGridMapCallback(
    const sensor_msgs::CompressedImageConstPtr &fs_grid_map_in) {
  if (!fs_grid_map_in->data.empty()) {
    fs_grid_map_ =
        cv::imdecode(cv::Mat(fs_grid_map_in->data), CV_LOAD_IMAGE_UNCHANGED);
    cv::threshold(fs_grid_map_, fs_grid_map_, 63, 255, CV_THRESH_BINARY);

    cv::Mat kernel_dilate =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::Mat local_map;
    cv_bridge::toCvCopy(fs_grid_map_in, "mono8")->image.copyTo(local_map);
    cv::dilate(local_map, dilate_fs_grid_map_, kernel_dilate);

    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "mono8", fs_grid_map_)
            .toImageMsg();
    pub_fs_grid_map_.publish(msg);
  }
}
}  // namespace planning
