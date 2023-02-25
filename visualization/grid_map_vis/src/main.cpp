#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv/cv.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cv_bridge/cv_bridge.h"
#include "cyber_msgs/Heading.h"
#include "cyber_msgs/LocalizationEstimate.h"
#include "sensor_msgs/Image.h"
// USB Key，只在生成发布版的时候才解注释
// #include "../../../common/Pwd_8/SoftkeyPWD.h"

class GridMapVis {
 public:
  GridMapVis(ros::NodeHandle* pnh);
  void UpdateLocalMap(const sensor_msgs::CompressedImageConstPtr& map_in);
  void UpdateFsLocalMap(const sensor_msgs::CompressedImageConstPtr& fs_map_in);
  void PubOccMap(const cv::Mat* map_in, std::string frame_id,
                 const std_msgs::Header& header_, int mode);

 private:
  ros::NodeHandle* pnh_;
  ros::Publisher pub_local_map_;
  ros::Publisher pub_dilated_local_map_;
  ros::Subscriber sub_local_map_;
  cv::Mat local_map_;
  cv::Mat dilated_local_map_;

  ros::Publisher pub_fs_local_map_;
  ros::Subscriber sub_fs_local_map_;
  cv::Mat fs_local_map_;

  ros::Subscriber sub_mag_;
  ros::Subscriber sub_heading_;
  ros::Subscriber sub_localization_;

  ros::Publisher pub_mag_degree_;
  ros::Publisher pub_local_degree_;
  ros::Publisher pub_heading_degree_;

  void MagCallback(const geometry_msgs::Vector3StampedConstPtr& msg_in);
  void HeadingCallback(const cyber_msgs::HeadingConstPtr& msg_in);
  void LocalCallback(const cyber_msgs::LocalizationEstimateConstPtr& msg_in);
};

GridMapVis::GridMapVis(ros::NodeHandle* pnh) : pnh_(pnh) {
  pub_local_map_ = pnh_->advertise<nav_msgs::OccupancyGrid>("local_map", 1);
  pub_dilated_local_map_ =
      pnh_->advertise<nav_msgs::OccupancyGrid>("dilated_local_map", 1);
  sub_local_map_ = pnh_->subscribe("/perception/local_grid_map/compressed", 1,
                                   &GridMapVis::UpdateLocalMap, this);
  pub_fs_local_map_ =
      pnh_->advertise<nav_msgs::OccupancyGrid>("fs_local_map", 5);
  sub_fs_local_map_ =
      pnh_->subscribe("/perception/freespace_grid_map/compressed", 5,
                      &GridMapVis::UpdateFsLocalMap, this);

  sub_mag_ = pnh_->subscribe("/imu/mag", 5, &GridMapVis::MagCallback, this);
  sub_heading_ =
      pnh_->subscribe("/strong/heading", 5, &GridMapVis::HeadingCallback, this);
  sub_localization_ = pnh_->subscribe("/localization/estimation", 5,
                                      &GridMapVis::LocalCallback, this);

  pub_mag_degree_ = pnh_->advertise<std_msgs::Float64>("/mag_degree", 5);
  pub_local_degree_ = pnh_->advertise<std_msgs::Float64>("/local_degree", 5);
  pub_heading_degree_ =
      pnh_->advertise<std_msgs::Float64>("/heading_degree", 5);
}

void GridMapVis::UpdateLocalMap(
    const sensor_msgs::CompressedImageConstPtr& map_in) {
  local_map_ = cv::imdecode(cv::Mat(map_in->data), CV_LOAD_IMAGE_UNCHANGED);
  threshold(local_map_, local_map_, 63, 255.0, CV_THRESH_BINARY);
  std_msgs::Header header_now = map_in->header;
  PubOccMap(&local_map_, "local_map", header_now, 0);

  cv::Mat kernel_dilate =
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  cv::Mat local_map;
  cv_bridge::toCvCopy(map_in, "mono8")->image.copyTo(local_map);
  cv::dilate(local_map, dilated_local_map_, kernel_dilate);
  threshold(dilated_local_map_, dilated_local_map_, 63, 255.0,
            CV_LOAD_IMAGE_UNCHANGED);
  PubOccMap(&dilated_local_map_, "local_map", header_now, 2);
}

void GridMapVis::UpdateFsLocalMap(
    const sensor_msgs::CompressedImageConstPtr& fs_map_in) {
  fs_local_map_ =
      cv::imdecode(cv::Mat(fs_map_in->data), CV_LOAD_IMAGE_UNCHANGED);
  threshold(fs_local_map_, fs_local_map_, 63, 255, CV_THRESH_BINARY);
  std_msgs::Header header_now = fs_map_in->header;
  PubOccMap(&fs_local_map_, "local_map", header_now, 1);
}

void GridMapVis::PubOccMap(const cv::Mat* map_in, std::string frame_id,
                           const std_msgs::Header& header_, int mode) {
  if (map_in->rows == 0 || map_in->cols == 0) {
    ROS_ERROR("NO MAP IN!!");
    return;
  }

  int newHeight = map_in->rows;
  int newWidth = map_in->cols;

  nav_msgs::OccupancyGrid::Ptr grid;
  grid.reset(new nav_msgs::OccupancyGrid);
  grid->header.frame_id = frame_id;
  grid->info.height = newHeight;
  grid->info.width = newWidth;
  grid->info.resolution = 0.1;
  grid->header.stamp = header_.stamp;

  for (int row = map_in->rows - 1; row >= 0; row--) {
    for (int col = 0; col < map_in->cols; col++) {
      if (map_in->at<uchar>(row, col) == 255)
        grid->data.emplace_back(100);  ///障碍物
      else if (map_in->at<uchar>(row, col) == 0)
        grid->data.emplace_back(0);  ///无障碍物
      else {
        grid->data.emplace_back(50);  ///未知
        // std::cout<<int(map_in->at<uchar>(row, col))<<std::endl;
      }
    }
  }
  if (mode == 2) {
    pub_dilated_local_map_.publish(*grid);
  } else if (mode == 1) {
    pub_fs_local_map_.publish(*grid);
  } else
    pub_local_map_.publish(*grid);
}

void GridMapVis::MagCallback(
    const geometry_msgs::Vector3StampedConstPtr& msg_in) {
  double mag_x = msg_in->vector.x;
  double mag_y = msg_in->vector.y;
  std_msgs::Float64 msg;
  msg.data = atan2(mag_y, mag_x) * 180.0 / M_PI + 180.0;
  pub_mag_degree_.publish(msg);
  std::cout << "mag_yaw : " << msg.data << std::endl;
}
void GridMapVis::HeadingCallback(const cyber_msgs::HeadingConstPtr& msg_in) {
  double heading = msg_in->data;
  std_msgs::Float64 msg;
  msg.data = heading * 180.0 / M_PI;
  pub_heading_degree_.publish(msg);
  std::cout << "heading yaw : " << msg.data << std::endl;
}
void GridMapVis::LocalCallback(
    const cyber_msgs::LocalizationEstimateConstPtr& msg_in) {
  double yaw = tf::getYaw(msg_in->pose.orientation);
  std_msgs::Float64 msg;
  msg.data = yaw * 180.0 / M_PI;
  pub_local_degree_.publish(msg);
  std::cout << "local yaw : " << msg.data << std::endl;
}

int main(int argc, char** argv) {
  // 只在生成发布版的时候才解注释，否则在自己电脑上无法调试
  // if(!checkUSBKey()) return 0;
  ros::init(argc, argv, "VisualizationGridMap");
  ros::NodeHandle pnh("~");
  GridMapVis vis(&pnh);
  ros::spin();
}