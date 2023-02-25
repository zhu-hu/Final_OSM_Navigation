#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <memory.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <time.h>

#include <string>

#include "FreeSpace.hpp"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PolygonStamped.h"
#include "grid_map.h"
#include "nav_msgs/GridCells.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Int8.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
ros::Publisher fs_points_pub;
ros::Publisher fs_pub;
ros::Publisher fs_distance_circle_pub;
ros::Publisher fs_distance_text_pub;
image_transport::Publisher pub_local_grid_map;

perception::LivoxFreeSpace livox_free_space;
perception::GridMap grid_map_;

std::vector<sensor_msgs::PointCloud2ConstPtr> recieved_pc_msgs;
sensor_msgs::PointCloud2ConstPtr this_pc_msg;
visualization_msgs::MarkerArray circles, texts;

bool is_background_pub = false, msg_type = true;
double recieved_pc_msg_time = 0;
bool recieved_pc_msg_flag = false;
bool pub_grid_map_flag = false;
uint8_t colors[19][3] = {
    {244, 67, 54},  {233, 30, 99},  {156, 39, 176}, {103, 58, 183},
    {63, 81, 181},  {33, 150, 243}, {3, 169, 244},  {0, 188, 212},
    {0, 150, 136},  {76, 175, 80},  {139, 195, 74}, {205, 220, 57},
    {255, 235, 59}, {255, 193, 7},  {255, 152, 0},  {25, 87, 34},
    {121, 85, 72},  {96, 125, 139}, {255, 100, 200}};
uint8_t colors_bg[10][3] = {{83, 134, 139}, {0, 139, 139},  {46, 139, 87},
                            {84, 139, 84},  {47, 79, 79},   {139, 117, 0},
                            {139, 10, 80},  {104, 34, 139}, {16, 78, 139},
                            {96, 123, 139}};

float height_offset = 0.0;
std_msgs::Header gheader;

namespace perception {
void PrepareBackground() {
  for (int dis = 50; dis < 500; dis = dis + 50) {
    visualization_msgs::Marker circle, text;
    circle.header.frame_id = "livox_frame";
    circle.header.stamp = ros::Time();
    circle.id = dis;
    circle.action = visualization_msgs::Marker::ADD;
    circle.type = visualization_msgs::Marker::LINE_STRIP;
    circle.lifetime = ros::Duration();
    circle.color.r = 0.5;
    circle.color.g = 0.5;
    circle.color.b = 0.5;
    circle.color.a = 1;
    circle.scale.x = 0.1;
    for (float i = 0; i <= 2 * 3.2; i = i + 0.1) {
      geometry_msgs::Point p;
      p.x = dis * cos(i);
      p.y = dis * sin(i);
      p.z = -2;
      circle.points.push_back(p);
    }
    circles.markers.push_back(circle);

    text.header.frame_id = "livox_frame";
    text.header.stamp = ros::Time();
    text.id = dis;
    text.action = visualization_msgs::Marker::ADD;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.lifetime = ros::Duration();
    text.color.r = 1;
    text.color.g = 1;
    text.color.b = 1;
    text.color.a = 1;
    text.scale.z = 5;
    text.pose.position.x = dis * cos(5.76);
    text.pose.position.y = dis * sin(5.76);
    text.pose.position.z = -1;
    text.pose.orientation.x = 0;
    text.pose.orientation.y = 0;
    text.pose.orientation.z = -0.25;
    text.pose.orientation.w = 0.96;
    text.text = std::to_string(dis) + "m";
    texts.markers.push_back(text);
  }
  visualization_msgs::Marker one_line;
  one_line.header.frame_id = "livox_frame";
  one_line.header.stamp = ros::Time();
  one_line.id = 500;
  one_line.action = visualization_msgs::Marker::ADD;
  one_line.type = visualization_msgs::Marker::LINE_STRIP;
  one_line.lifetime = ros::Duration();
  one_line.color.r = 0.5;
  one_line.color.g = 0.5;
  one_line.color.b = 0.5;
  one_line.color.a = 1;
  one_line.scale.x = 0.1;
  geometry_msgs::Point p1, p2;
  p1.x = 0;
  p1.y = 0;
  p1.z = -2;
  p2.x = 500;
  p2.y = 0;
  p2.z = -2;
  one_line.points.push_back(p1);
  one_line.points.push_back(p2);
  circles.markers.push_back(one_line);
}

void ApplyColorToPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &pc,
                            const pcl::PointCloud<pcl::PointXYZI> &input) {
  for (int i = 0; i < pc.points.size(); i++) {
    pc.points[i].x = input.points[i].x;
    pc.points[i].y = input.points[i].y;
    pc.points[i].z = input.points[i].z;
    if (input.points[i].intensity < 30.0) {
      int green = input.points[i].intensity * 255.0 / 30.0;
      pc.points[i].r = 0;
      pc.points[i].g = green & 0xff;
      pc.points[i].b = 0xff;
    } else if (input.points[i].intensity < 90.0) {
      int blue = (90 - input.points[i].intensity) * 255.0 / 60.0;
      pc.points[i].r = 0;
      pc.points[i].g = 0xff;
      pc.points[i].b = blue & 0xff;
    } else if (input.points[i].intensity < 150.0) {
      int red = (input.points[i].intensity - 90) * 255.0 / 60.0;
      pc.points[i].r = red & 0xff;
      pc.points[i].g = 0xff;
      pc.points[i].b = 0;
    } else {
      int green = (255 - input.points[i].intensity) * 255.0 / (256 - 150);
      pc.points[i].r = 0xff;
      pc.points[i].g = green & 0xff;
      pc.points[i].b = 0;
    }
  }
}

void GenerateGridMap(const pcl::PointCloud<pcl::PointXYZI> &pc,
                     const ros::Time &stamp) {
  cv::Mat roi_grid_map;
  grid_map_.generate_grid_map(pc, roi_grid_map);
  sensor_msgs::ImagePtr image_msg =
      cv_bridge::CvImage(std_msgs::Header(), "mono8", roi_grid_map)
          .toImageMsg();
  image_msg->header.stamp = stamp;
  image_msg->header.frame_id = "livox_frame";
  if (pub_grid_map_flag)
    pub_local_grid_map.publish(image_msg);
}

void GenerateFreeSpace(pcl::PointCloud<pcl::PointXYZI> &pc) {
  clock_t t0, t1, t2;
  t0 = clock();

  int dnum = pc.points.size();
  std::cout << "Point cloud size: " << dnum << std::endl;

  float *data = (float *)calloc(dnum * 4, sizeof(float));
  std::vector<float> free_space;
  for (int p = 0; p < dnum; ++p) {
    data[p * 4 + 0] = pc.points[p].x;
    data[p * 4 + 1] = pc.points[p].y;
    data[p * 4 + 2] = pc.points[p].z;
    data[p * 4 + 3] = pc.points[p].intensity;
  }
  livox_free_space.GenerateFreeSpace(data, dnum, free_space);
  t1 = clock();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud->clear();
  cloud->width = dnum;
  cloud->height = 1;
  cloud->is_dense = false;
  cloud->points.resize(cloud->width * cloud->height);
  ApplyColorToPointCloud(*cloud, pc);
  sensor_msgs::PointCloud2 msg2;
  pcl::toROSMsg(*cloud, msg2);

  msg2.header.stamp = gheader.stamp;
  msg2.header.frame_id = "livox_frame";
  fs_points_pub.publish(msg2);

  pcl::PointCloud<pcl::PointXYZI> fs;
  fs.clear();
  for (int i = 0; i < free_space.size(); i += 3) {
    pcl::PointXYZI p;
    p.x = free_space[i];
    p.y = free_space[i + 1];
    p.z = 0;
    p.intensity = free_space[i + 2];
    fs.points.push_back(p);
  }
  sensor_msgs::PointCloud2 msg3;
  pcl::toROSMsg(fs, msg3);

  msg3.header.stamp = gheader.stamp;
  msg3.header.frame_id = "livox_frame";
  fs_pub.publish(msg3);
  std::vector<float>().swap(free_space);
  t2 = clock();
  GenerateGridMap(fs, gheader.stamp);

  printf("\n\n");
  printf("Total Time: %f, FreeSpace: %f, Publish Results: %f\n\n",
         1000.0 * (t2 - t0) / CLOCKS_PER_SEC,
         1000.0 * (t1 - t0) / CLOCKS_PER_SEC,
         1000.0 * (t2 - t1) / CLOCKS_PER_SEC);
  printf("---------------------------------------------\n\n");
  free(data);
}

void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  ROS_INFO("Recieved pointcloud: secs = %u, nsecs = %u",
           cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nsec);
  // this_pc_msg = cloud_msg;
  this_pc_msg = cloud_msg;
  recieved_pc_msg_time = cloud_msg->header.stamp.toSec();
  recieved_pc_msg_flag = true;
}

void PerceptionModeCloudCallback(const std_msgs::Int8ConstPtr &msg) {
  const int &m = msg->data;
  ROS_INFO("Recieved perception mode: %d", m);
  if (m == 1 || m == 3) {
    pub_grid_map_flag = true;
  } else {
    pub_grid_map_flag = false;
  }
}
} // namespace perception

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_perception_free_space_node");

  ros::NodeHandle n;
  perception::GridMapParams params;
  n.getParam("height_offset", height_offset);
  n.getParam("gird_map_x_min", params.roi_params.min_x);
  n.getParam("gird_map_x_max", params.roi_params.max_x);
  n.getParam("gird_map_y_min", params.roi_params.min_y);
  n.getParam("gird_map_y_max", params.roi_params.max_y);
  n.getParam("gird_map_pixel", params.roi_params.pixel_scale);
  grid_map_.setParams(params);

  ros::Subscriber sub_pc;
  sub_pc = n.subscribe("/driver/livox/point_cloud", 10,
                       perception::PointCloudCallback);
  ros::Subscriber sub_mode;
  sub_mode = n.subscribe("/perception_mode", 10,
                         perception::PerceptionModeCloudCallback);
  fs_points_pub =
      n.advertise<sensor_msgs::PointCloud2>("/fs_pointcloud/pointcloud", 10);
  fs_distance_circle_pub =
      n.advertise<visualization_msgs::MarkerArray>("fs_marker/circle", 10);
  fs_distance_text_pub =
      n.advertise<visualization_msgs::MarkerArray>("fs_marker/dis_text", 10);
  fs_pub =
      n.advertise<sensor_msgs::PointCloud2>("/fs_pointcloud/free_space", 10);
  image_transport::ImageTransport it(n);
  pub_local_grid_map = it.advertise("/perception/freespace_grid_map", 2);

  ros::Rate rate(20);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    if (!is_background_pub) {
      perception::PrepareBackground();
      is_background_pub = true;
    }
    fs_distance_circle_pub.publish(circles);
    fs_distance_text_pub.publish(texts);
    pcl::PointCloud<pcl::PointXYZI> pc;

    if (recieved_pc_msg_flag) {
      pcl::fromROSMsg(*this_pc_msg, pc);
      for (int i = 0; i < pc.points.size(); i++)
        pc.points[i].z = pc.points[i].z + height_offset;
      gheader = this_pc_msg->header;
      perception::GenerateFreeSpace(pc);
      recieved_pc_msg_flag = false;
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}