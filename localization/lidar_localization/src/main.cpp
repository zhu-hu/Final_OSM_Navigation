#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <mrpt_bridge/mrpt_bridge.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "common/struct/StateEstimation.h"
#include "common/util/coor_conversion.h"
#include "common/util/pointcloud_util.h"
#include "cyber_msgs/Heading.h"
#include "cyber_msgs/VehicleSpeedFeedback.h"
#include "cyber_msgs/GPGGA_MSG.h"
#include "grid_map_localization.h"

localization::Pose3D lidar_cali_info;
localization::Pose3D gps_cali_info;
localization::BoundingBox cloud_boundingbox;
localization::BoundingBox loc_cloud_boundingbox;
localization::BoundingBox CLbox;
localization::Pose2D last_pose;
bool has_CL;
//栅格地图参数
int grid_map_size;
double grid_map_resolution;
double voxelgrid_filter_size;
std::string data_folder_path;
std::string grid_map_folder_path;
localization::GridMapLocalization *gml = nullptr;

//起始方式
int initialization_mode = 0;
double large_map_center_x = 0;
double large_map_center_y = 0;
double large_map_angle = 0;
// utm参数
double utm_origin_x = 0;
double utm_origin_y = 0;
double utm_origin_z = 0;
// char utm_band = 82;
// u_int8_t utm_zone = 49;
int utm_band;
int utm_zone;

//更新判断标志
bool gps_fix_updated_flag = false;
bool heading_updated_flag = false;
bool speed_updated_flag = false;
bool pointcloud_update_flag = false;
bool init_flag = false;
int delay_num = 0;
int delay_num_ = 0;
int stay_count = 0;
bool slam_area_flag = false;
bool slam_area_flag_changed = false;
bool restart_mode = false;
bool gps_heading_err_flag = false;
double gloabl_angle = 0;
bool pose_update = false;

//拟合轨迹直线推断航向
std::list<localization::Pose2D> pose_list;
bool use_traj_heading = false;
double speed = 0;

//边界切换模式
bool use_roi = false;
std::string slam_area_file;
std::string map_file_name;

//发布topic
std::string gps_fix_topic;
std::string gps_heading_topic;
std::string speed_topic;
std::string laser_topic;
std::string gps_status_topic;

ros::Publisher *map_publisher;
ros::Publisher *pointcloud_publisher;
ros::Publisher *odom_publisher;
ros::Publisher *slam_fix_publisher;
ros::Publisher *slam_heading_publisher;
ros::Publisher *marker_publisher;  // slam区域显示

//坐标系维护发布
tf::TransformBroadcaster *tf_broadcaster;
tf::StampedTransform *registered_transform;

// gps位置和全局位置
localization::StateEstimation gps_posture;
localization::StateEstimation global_posture;

// ui初始化界面
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
localization::Pose2D init_pose;
tf::Vector3 global_position(0.0, 0.0, 10);
tf::Vector3 global_orientaion(0.0, 0.0, 0.0);
float global_scale = 30.0;

std::ofstream pose_file_;
bool record_data = 0;
int gps_status = 0;
int gps_area_cnt = 0;
int error_cnt=0;

void StrongFixCallback(const sensor_msgs::NavSatFixConstPtr &fix_msg);

void StrongHeadingCallback(const cyber_msgs::HeadingConstPtr &heading_msg);

void StrongStatusCallback(const cyber_msgs::GPGGA_MSGConstPtr &gps_status_msg);

void SpeedCallback(const cyber_msgs::VehicleSpeedFeedbackConstPtr &vel_msg);

void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

void PointCloudPublishingCallback(const ros::TimerEvent &);

void MapPublishingCallback(const ros::TimerEvent &);

visualization_msgs::InteractiveMarkerControl &MakeArrowControl(
    visualization_msgs::InteractiveMarker &msg);

visualization_msgs::InteractiveMarker CreateMarker(tf::Vector3 pose_in,
                                                   tf::Vector3 pose_ori,
                                                   float scale);

void ProcessFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

void SlamAreaPublishingCallback(const ros::TimerEvent &);

std::vector<float> LineFitLeastSquares(
    std::list<localization::Pose2D> pose_list, int data_n);

int main(int argc, char **argv) {
  ros::init(argc, argv, "LidarLocalization");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  //设置topic
  pnh.param<std::string>("gps_fix_topic", gps_fix_topic, "/strong/fix");
  pnh.param<std::string>("gps_heading_topic", gps_heading_topic,
                         "/strong/heading");
  pnh.param<std::string>("speed_topic", speed_topic, "/e100/speed_feedback");
  pnh.param<std::string>("laser_topic", laser_topic,
                         "/driver/livox/point_cloud");
  pnh.param<std::string>("gps_status_topic", gps_status_topic , "/strong/raw_data");
  //起始方式
  pnh.param<int>("initialization_mode", initialization_mode, 0);
  pnh.param<double>("large_map_center_x", large_map_center_x, 0);
  pnh.param<double>("large_map_center_y", large_map_center_y, 0);
  pnh.param<double>("large_map_angle", large_map_angle, 0);
  // utm坐标原点设定
  pnh.param("GLOBAL_ZERO_X_", utm_origin_x, utm_origin_x);
  pnh.param("GLOBAL_ZERO_Y_", utm_origin_y, utm_origin_y);
  pnh.param("GLOBAL_ZERO_Z_", utm_origin_z, utm_origin_z);
  pnh.param("utm_origin_band", utm_band, utm_band);
  pnh.param("utm_origin_zone", utm_zone, utm_zone);
  //点云截取范围
  pnh.param("cloud_boudingbox/cloud_boundingbox_max_x", cloud_boundingbox.max.x, cloud_boundingbox.max.x);
  pnh.param("cloud_boudingbox/cloud_boundingbox_min_x", cloud_boundingbox.min.x, cloud_boundingbox.min.x);
  pnh.param("cloud_boudingbox/cloud_boundingbox_max_y", cloud_boundingbox.max.y, cloud_boundingbox.max.y);
  pnh.param("cloud_boudingbox/cloud_boundingbox_min_y", cloud_boundingbox.min.y, cloud_boundingbox.min.y);
  pnh.param("cloud_boudingbox/cloud_boundingbox_max_z", cloud_boundingbox.max.z, cloud_boundingbox.max.z);
  pnh.param("cloud_boudingbox/cloud_boundingbox_min_z", cloud_boundingbox.min.z, cloud_boundingbox.min.z);

  pnh.param("has_CL", has_CL, has_CL);
  pnh.param("loc_cloud_boudingbox/loc_cloud_boundingbox_max_x", loc_cloud_boundingbox.max.x,
                    loc_cloud_boundingbox.max.x);
  pnh.param("loc_cloud_boudingbox/loc_cloud_boundingbox_min_x", loc_cloud_boundingbox.min.x,
                    loc_cloud_boundingbox.min.x);
  pnh.param("loc_cloud_boudingbox/loc_cloud_boundingbox_max_y", loc_cloud_boundingbox.max.y,
                    loc_cloud_boundingbox.max.y);
  pnh.param("loc_cloud_boudingbox/loc_cloud_boundingbox_min_y", loc_cloud_boundingbox.min.y,
                    loc_cloud_boundingbox.min.y);
  pnh.param("loc_cloud_boudingbox/loc_cloud_boundingbox_max_z", loc_cloud_boundingbox.max.z,
                    loc_cloud_boundingbox.max.z);
  pnh.param("loc_cloud_boudingbox/loc_cloud_boundingbox_min_z", loc_cloud_boundingbox.min.z,
                    loc_cloud_boundingbox.min.z);

  //激光雷达和车辆中心的标定值
  pnh.param("lidar_to_vehicle/lidar_to_vehicle_x", lidar_cali_info.x, lidar_cali_info.x);
  pnh.param("lidar_to_vehicle/lidar_to_vehicle_y", lidar_cali_info.y, lidar_cali_info.y);
  pnh.param("lidar_to_vehicle/lidar_to_vehicle_z", lidar_cali_info.z, lidar_cali_info.z);
  pnh.param("lidar_to_vehicle/lidar_to_vehicle_rx", lidar_cali_info.rx, lidar_cali_info.rx);
  pnh.param("lidar_to_vehicle/lidar_to_vehicle_ry", lidar_cali_info.ry, lidar_cali_info.ry);
  pnh.param("lidar_to_vehicle/lidar_to_vehicle_rz", lidar_cali_info.rz, lidar_cali_info.rz);
  //激光雷达和车辆中心的标定值
  pnh.param("gps_to_vehicle/gps_to_vehicle_x", gps_cali_info.x, gps_cali_info.x);
  pnh.param("gps_to_vehicle/gps_to_vehicle_y", gps_cali_info.y, gps_cali_info.y);
  //降采样值
  pnh.param("voxelgrid_filter_size", voxelgrid_filter_size, voxelgrid_filter_size);
  pnh.param("map_file_name", map_file_name , map_file_name);
  //栅格地图路径
  pnh.param<int>("map_size", grid_map_size, 5);
  pnh.param<double>("map_resolution", grid_map_resolution, 0.1);
  pnh.param<std::string>("data_folder_path", data_folder_path,
                         "./data");
  // slam运行区域
  pnh.param<bool>("use_slam_area", use_roi, 0);
  pnh.param<std::string>("slam_area_file", slam_area_file,
                         "/grid_map_folder_path/slam_area.txt");
  pnh.param("delay_num_", delay_num_, delay_num_);
  pnh.param("use_traj_heading", use_traj_heading, use_traj_heading);
  pnh.param("restart_mode", restart_mode, restart_mode);
  pnh.param("error_cnt", error_cnt, error_cnt);

  grid_map_folder_path = data_folder_path+map_file_name+"/";
  slam_area_file = data_folder_path+map_file_name+"/slam_area.txt";

  CLbox.min.x = loc_cloud_boundingbox.min.x;
  CLbox.min.y = loc_cloud_boundingbox.min.y;
  CLbox.min.z = loc_cloud_boundingbox.min.z;
  CLbox.max.x = loc_cloud_boundingbox.max.x;
  CLbox.max.y = loc_cloud_boundingbox.max.y;
  CLbox.max.z = loc_cloud_boundingbox.max.z;
  delay_num = delay_num_;
  if (initialization_mode > 1) {
    std::cout << "initialization_mode error!" << std::endl;
    return 0;
  }

  gml = new localization::GridMapLocalization();
  gml->SetLidarCaliInfo(lidar_cali_info);
  gml->SetCloudBoundingBox(cloud_boundingbox);
  gml->SetVoxelgridFilterSize(voxelgrid_filter_size);
  gml->SetGridMapInfo(grid_map_size, grid_map_resolution, grid_map_folder_path);
  if (use_roi) {
    if (!gml->LoadSlamArea(slam_area_file)) {
      std::cout << "[ERROR]Can not load slam area file!" << std::endl;
    }
  }
  if (initialization_mode == 1) {
    gml->LoadGridMap(large_map_center_x, large_map_center_y);
    global_posture.pose.position.x = large_map_center_x;
    global_posture.pose.position.y = large_map_center_y;
    init_pose.x = large_map_center_x;
    init_pose.y = large_map_center_y;
    init_pose.phi = large_map_angle;
    gml->UpdatePointCloudPose(init_pose);
  }

  //记录slam定位结果在txt文件中，易于分析
  if (record_data)
    pose_file_.open("/home/lzg/Desktop/localization_v2_ws/odom.txt");

  tf_broadcaster = new tf::TransformBroadcaster;
  registered_transform = new tf::StampedTransform;
  registered_transform->frame_id_ = "map";
  registered_transform->child_frame_id_ = "lidar";

  ros::Subscriber gps_fix_subscriber =
      nh.subscribe(gps_fix_topic, 1, StrongFixCallback);
  ros::Subscriber gps_heading_subscriber =
      nh.subscribe(gps_heading_topic, 1, StrongHeadingCallback);
  ros::Subscriber gps_status_subscriber = nh.subscribe(gps_status_topic, 1 , StrongStatusCallback);
  ros::Subscriber speed_subscriber =
      nh.subscribe(speed_topic, 1, SpeedCallback);
  ros::Subscriber pointcloud_subscriber =
      nh.subscribe(laser_topic, 10, PointCloudCallback);

  map_publisher =
      new ros::Publisher(pnh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1));
  pointcloud_publisher = new ros::Publisher(
      pnh.advertise<sensor_msgs::PointCloud2>("registered_cloud", 10));
  odom_publisher = new ros::Publisher(
      pnh.advertise<nav_msgs::Odometry>("map_matching_result/utm", 1));
  slam_fix_publisher = new ros::Publisher(
      pnh.advertise<sensor_msgs::NavSatFix>("/slam/gps/fix", 1));
  slam_heading_publisher = new ros::Publisher(
      pnh.advertise<sensor_msgs::Imu>("/slam/gps/heading", 1));
  marker_publisher =
      new ros::Publisher(pnh.advertise<visualization_msgs::MarkerArray>(
          "slam_area_visualization_marker", 20));

  ros::Timer point_cloud_publishing_timer =
      nh.createTimer(ros::Duration(0.05), PointCloudPublishingCallback);
  ros::Timer map_publishing_timer =
      nh.createTimer(ros::Duration(1), MapPublishingCallback);
  ros::Timer slam_area_timer;
  if (use_roi) {
    slam_area_timer =
        nh.createTimer(ros::Duration(1), SlamAreaPublishingCallback);
  }
  if (initialization_mode == 1) {
    server.reset(new interactive_markers::InteractiveMarkerServer(
        "LidarLocalization", "", false));

    menu_handler.insert("Apply", &ProcessFeedback);
    menu_handler.insert("Zoom in", &ProcessFeedback);
    menu_handler.insert("Zoom out", &ProcessFeedback);

    global_position[0] = large_map_center_x;
    global_position[1] = large_map_center_y;
    global_orientaion[2] = large_map_angle;

    visualization_msgs::InteractiveMarker int_marker =
        CreateMarker(global_position, global_orientaion, global_scale);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &ProcessFeedback);
    menu_handler.apply(*server, int_marker.name);
    server->applyChanges();
  }
  ros::MultiThreadedSpinner spinner(0);
  spinner.spin();

  return 0;
}

void StrongStatusCallback(const cyber_msgs::GPGGA_MSGConstPtr &gps_status_msg){
  gps_status = gps_status_msg->status;
}

//使用gps时提供全局位置更新并用gps更新轨迹。
void StrongFixCallback(const sensor_msgs::NavSatFixConstPtr &fix_msg) {
  localization::Point utm_ori(utm_origin_x, utm_origin_y, utm_origin_z);
  localization::util::FixmsgDecode(&gps_posture, fix_msg, gps_cali_info.x,
                                   gps_cali_info.y, utm_ori);
  // std::cout<< "gps_time="<<fix_msg->header.stamp.toSec()<<std::endl;

//    std::cout << std::fixed << std::setprecision(4) << " Pose_X "
//             << gps_posture.pose.position.x << " Pose_Y "
//             << gps_posture.pose.position.y << " Pose_RZ "<<'\n';
  double temp_rx, temp_ry, temp_rz;
  //通过轨迹拟合直线得到航向
  if (!heading_updated_flag) return;
  tf::Matrix3x3(tf::Quaternion(gps_posture.pose.orientation.x,
                               gps_posture.pose.orientation.y,
                               gps_posture.pose.orientation.z,
                               gps_posture.pose.orientation.w))
      .getRPY(temp_rx, temp_ry, temp_rz);
  if (use_traj_heading) {
    localization::Pose2D pose_point;
    pose_point.x = gps_posture.pose.position.x;
    pose_point.y = gps_posture.pose.position.y;
    pose_list.push_back(pose_point);
    double heading_predict = 9999;
    std::vector<float> v_res;
    if (pose_list.size() > 100) {
      pose_list.pop_front();
      v_res = LineFitLeastSquares(pose_list, pose_list.size());
      heading_predict = std::atan(v_res[0]);
      if (heading_predict >= M_PI)
        heading_predict -= 2 * M_PI;
      else if (heading_predict <= -M_PI)
        heading_predict += 2 * M_PI;
      if ((heading_predict - temp_rz) >= M_PI / 2)
        heading_predict -= M_PI;
      else if ((heading_predict - temp_rz) <= -M_PI / 2)
        heading_predict += M_PI;
    }
    double heading;
    if (heading_predict == 9999 || speed == 0)
      heading = temp_rz;
    else
      heading = heading_predict;
    temp_rz = heading;
  }

  //使用gps时使用gps更新全局位置，若没有地图先加载地图
  //slam区域通过gps初始化
  if (init_flag && slam_area_flag && delay_num > 0) {
    global_posture.pose = gps_posture.pose;
    last_pose.x = global_posture.pose.position.x;
    last_pose.y = global_posture.pose.position.y;
    gml->SetMatchingInitPose(global_posture.pose.position.x,
                             global_posture.pose.position.y, temp_rz);
    gloabl_angle = temp_rz;
    delay_num--;
    std::cout << "[gps] init slam by gps." << '\n';
  }

  //定位恢复模块,在gps区域恢复定位
  if (init_flag && slam_area_flag && restart_mode) {
    if(!gml->IsInSlamArea(gps_posture.pose)&&gps_status==4){
      gps_area_cnt++;
    }else if(gps_area_cnt>0){
      gps_area_cnt--;
    }
    if(gps_area_cnt>20){
      global_posture.pose = gps_posture.pose;
      global_posture.timestamp = gps_posture.timestamp;
      gps_area_cnt = 0;
      slam_area_flag = false;
    }
  }

  //在gps区域
  if (init_flag && !slam_area_flag) {
    delay_num = delay_num_;
    global_posture.pose = gps_posture.pose;
    gml->SetMatchingInitPose(global_posture.pose.position.x,
                             global_posture.pose.position.y, temp_rz);
    gloabl_angle = temp_rz;
    nav_msgs::Odometry OutResultMsg = localization::util::LocaRestoOdom(
        gps_posture.pose, "map", fix_msg->header.stamp);
    odom_publisher->publish(OutResultMsg);
    std::cout << "[gps] update by gps." << '\n';
  }
  if ((initialization_mode == 0) && !init_flag) {
    global_posture.pose = gps_posture.pose;
    gml->LoadGridMap(global_posture.pose.position.x,
                     global_posture.pose.position.y);
    gml->SetMatchingInitPose(global_posture.pose.position.x,
                             global_posture.pose.position.y, temp_rz);
    if (gml->IsInSlamArea(global_posture.pose))
      slam_area_flag = true;
    else
      slam_area_flag = false;
    init_flag = true;
  }

  gps_fix_updated_flag = true;
}

void StrongHeadingCallback(const cyber_msgs::HeadingConstPtr &heading_msg) {
  double temp_rx = 0.0, temp_ry = 0.0, temp_rz;
  double yaw = heading_msg->data;
  if (yaw >= M_PI)
    yaw -= 2 * M_PI;
  else if (yaw <= -M_PI)
    yaw += 2 * M_PI;

  //用来判断航向是否存在错误，辅助判断更新位置
  double obs_yaw = yaw;
  if (obs_yaw < -M_PI || obs_yaw > M_PI || std::isnan(obs_yaw) ||
      std::isinf(obs_yaw))
    gps_heading_err_flag = true;
  else
    gps_heading_err_flag = false;

  // std::cout<<"    angle="<<obs_yaw/M_PI*180<<std::endl;
  temp_rz = yaw;
  if (std::isnan(yaw)) {
    heading_updated_flag = true;
    return;
  }
  if (!init_flag)
    localization::util::RPYtoStaEsOri(&global_posture, temp_rx, temp_ry,
                                      temp_rz);
  localization::util::RPYtoStaEsOri(&gps_posture, temp_rx, temp_ry, temp_rz);
  heading_updated_flag = true;
}

void SpeedCallback(const cyber_msgs::VehicleSpeedFeedbackConstPtr &vel_msg) {
  speed = abs(double(vel_msg->speed_cmps) * 0.01);
  if(std::isnan(speed))
    ROS_ERROR("speed_error");
  localization::util::CalStaEsTwi(&global_posture, speed);
  speed_updated_flag = true;
}

void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  localization::util::PointCloudTypePtr point_cloud_input(
      new localization::util::PointCloudType());
  pcl::fromROSMsg(*cloud_msg, *point_cloud_input);
  //更新点云和时间
  gml->UpdatePointCloud(point_cloud_input, cloud_msg->header.stamp.toSec());

  //河西ioc区域定位高度改变
  if (has_CL&&gml->IsInCL(global_posture.pose)) {
    gml->SetCloudBoundingBox(CLbox);
  } else {
    gml->SetCloudBoundingBox(cloud_boundingbox);
  }
  gml->PreprocessPointCloud();
 
  if (init_flag && gps_fix_updated_flag && heading_updated_flag &&
      slam_area_flag) {
     //速度补偿
    if (pose_update && (delay_num == 0)) {
      double temp_x = 0;
      double temp_y = 0;
      temp_x = last_pose.x + speed * cos(gloabl_angle) * 0.05;
      temp_y = last_pose.y + speed * sin(gloabl_angle) * 0.05;
      gml->SetMatchingInitPose(temp_x, temp_y);
      pose_update = false;
    }
    gml->PointCloudMatch();
    const localization::Pose2DStamped *localization_result =
        gml->GetPointCloudMatchingResult();

    if (localization_result != nullptr) {

      //记录slam定位结果在txt文件中，易于分析
      if (record_data) {
        double temp_zero = 0;
        pose_file_ << std::fixed << std::setprecision(4)
                   << cloud_msg->header.stamp.toSec() << '\t'
                   << localization_result->pose.x << '\t'
                   << localization_result->pose.y << '\t' << temp_zero << '\t'
                   << temp_zero << '\t' << temp_zero << '\t'
                   << localization_result->pose.phi << '\t' << std::endl;
      }

      //长廊效应临时解决措施
    //    if(gps_status == 4){
    //     double diff_x = gps_posture.pose.position.x - localization_result->pose.x;
    //     double diff_y = gps_posture.pose.position.y - localization_result->pose.y;
    //     double heading_diff = cos(localization_result->pose.phi)*diff_x + sin(localization_result->pose.phi)*diff_y;
    //     //std::cout<<"  heading_diff="<<heading_diff<<std::endl;
    //     if(heading_diff>0.6||heading_diff<-0.6){
    //       global_posture.pose = gps_posture.pose;
    //       gml->SetMatchingInitPose(global_posture.pose.position.x,
    //                          global_posture.pose.position.y);

    //     }else{
    //       localization::util::UpdateGlobalPosition(&global_posture,
    //                                            localization_result); 
    //     }
    //   }else
      localization::util::UpdateGlobalPosition(&global_posture,
                                               localization_result);

      last_pose.x = global_posture.pose.position.x;
      last_pose.y = global_posture.pose.position.y;
      gloabl_angle = localization_result->pose.phi;
      pose_update = true;

      localization::Point utm_ori(utm_origin_x, utm_origin_y, utm_origin_z);
      sensor_msgs::NavSatFix slam_fix_out = localization::util::LocaRestoNav(
          localization_result, utm_ori, char(utm_band), u_int8_t(utm_zone));
      slam_fix_publisher->publish(slam_fix_out);
      sensor_msgs::Imu slam_heading_out =
          localization::util::LocaRestoImu(localization_result);
      slam_heading_publisher->publish(slam_heading_out);
      nav_msgs::Odometry OutResultMsg =
          localization::util::LocaRestoOdom(localization_result, "map");
      odom_publisher->publish(OutResultMsg);
    }
  }
  pointcloud_update_flag = true;
}

void PointCloudPublishingCallback(const ros::TimerEvent &) {
  if ((!init_flag && pointcloud_update_flag) ||
      (init_flag && pointcloud_update_flag && gml->IsPointCloudUpdate)) {
    if ((initialization_mode == 0 && slam_area_flag) ||
        (initialization_mode == 1 &&
         (!init_flag || (init_flag && slam_area_flag)))) {
      const localization::Pose2DStamped *cloud_matching_result =
          gml->GetPointCloudMatchingResult();
      if (cloud_matching_result != nullptr) {
        registered_transform->stamp_ =
            ros::Time().fromSec(cloud_matching_result->timestamp);
        geometry_msgs::Quaternion quat =
            tf::createQuaternionMsgFromRollPitchYaw(
                0, 0, cloud_matching_result->pose.phi);
        registered_transform->setRotation(
            tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
        registered_transform->setOrigin(tf::Vector3(
            cloud_matching_result->pose.x, cloud_matching_result->pose.y, 0));
        tf_broadcaster->sendTransform(*registered_transform);
        sensor_msgs::PointCloud2 cloud_msg = gml->GetPointCloud("lidar");
        cloud_msg.header.stamp =
            ros::Time().fromSec(cloud_matching_result->timestamp);
        pointcloud_publisher->publish(cloud_msg);
      }
    }
    gml->IsPointCloudUpdate = false;
  }

  //slam区域和gps区域切换
  if (use_roi && init_flag && !std::isnan(global_posture.pose.position.x) &&
      !std::isnan(global_posture.pose.position.y)) {
    if (slam_area_flag_changed) {
      stay_count = 10;
      slam_area_flag_changed = false;
    }
    if (stay_count > 0) {
      stay_count--;
      return;
    }
    if (gml->IsInSlamArea(global_posture.pose)) {
      if (slam_area_flag == false) slam_area_flag_changed = true;
      slam_area_flag = true;
    } else if (!gml->IsInSlamArea(global_posture.pose) &&
               !gps_heading_err_flag) {
      if (slam_area_flag == true) slam_area_flag_changed = true;
      slam_area_flag = false;
    }
  }
}

void MapPublishingCallback(const ros::TimerEvent &) {
  nav_msgs::OccupancyGrid map_msg = gml->GetRosGridMap("map");
  if (map_msg.data.size() > 0) {
    // std::cout<<"    global_posture:x="<<global_posture.pose.position.x<<"
    // y="<<global_posture.pose.position.y<<std::endl;
    if (std::isnan(global_posture.pose.position.x) ||
        std::isnan(global_posture.pose.position.y))
      return;
    gml->UpdateMap(global_posture.pose.position.x,
                   global_posture.pose.position.y);
    if (map_msg.data.size() > 0) {
      map_publisher->publish(map_msg);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////

visualization_msgs::InteractiveMarkerControl &MakeArrowControl(
    visualization_msgs::InteractiveMarker &msg) {
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = msg.scale * 0.5;
  marker.scale.y = msg.scale * 0.1;
  marker.scale.z = msg.scale * 0.1;
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 1;
  marker.color.a = 1.0;

  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(marker);
  msg.controls.push_back(control);

  return msg.controls.back();
}

visualization_msgs::InteractiveMarker CreateMarker(tf::Vector3 pose_in,
                                                   tf::Vector3 pose_ori,
                                                   float scale) {
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  tf::Vector3 position(pose_in[0], pose_in[1], pose_in[2]);
  geometry_msgs::Quaternion quat =
      tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose_ori[2]);
  int_marker.pose.orientation.x = quat.x;
  int_marker.pose.orientation.w = quat.w;
  int_marker.pose.orientation.y = quat.y;
  int_marker.pose.orientation.z = quat.z;

  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = scale;

  int_marker.name = "initializer";
  int_marker.description = "Initializer";

  MakeArrowControl(int_marker);

  visualization_msgs::InteractiveMarkerControl control1, control2;

  tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
  tf::quaternionTFToMsg(orien.normalize(), control1.orientation);
  control1.name = "move_x";
  control1.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control1);

  orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
  tf::quaternionTFToMsg(orien.normalize(), control1.orientation);
  control1.name = "rotation_z";
  control1.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control1);

  orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
  tf::quaternionTFToMsg(orien.normalize(), control1.orientation);
  control1.name = "move_y";
  control1.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control1);

  control2.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MENU;
  control2.name = "initializer_menu";

  control2.always_visible = true;
  int_marker.controls.push_back(control2);

  return int_marker;
}

void ProcessFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
    << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;

  switch (feedback->event_type) {
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT: {
      if (feedback->menu_entry_id == 1) {
        gml->SetMatchingInitPose(init_pose.x, init_pose.y,
                                 init_pose.phi);  //设置匹配起始位置
        slam_area_flag = true;
        delay_num = 0;
        init_flag = true;
        server.reset();
        return;
      }

      if (feedback->menu_entry_id == 2) {
        global_scale *= 3.0;
        visualization_msgs::InteractiveMarker int_marker =
            CreateMarker(global_position, global_orientaion, global_scale);
        server->clear();
        server->insert(int_marker);
        menu_handler.apply(*server, int_marker.name);
      }

      if (feedback->menu_entry_id == 3) {
        global_scale /= 3.0;
        visualization_msgs::InteractiveMarker int_marker =
            CreateMarker(global_position, global_orientaion, global_scale);
        server->clear();
        server->insert(int_marker);
        menu_handler.apply(*server, int_marker.name);
      }
    } break;

    //如果拖动marker，点云位置随着改变
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE: {
      double roll, pitch, yaw;
      tf::Matrix3x3(tf::Quaternion(feedback->pose.orientation.x,
                                   feedback->pose.orientation.y,
                                   feedback->pose.orientation.z,
                                   feedback->pose.orientation.w))
          .getRPY(roll, pitch, yaw);
      global_posture.pose.position.x = feedback->pose.position.x;
      global_posture.pose.position.y = feedback->pose.position.y;

      init_pose.x = feedback->pose.position.x;
      init_pose.y = feedback->pose.position.y;
      std::cout << " init_pose.x=" << init_pose.x << std::endl;
      std::cout << " init_pose.y=" << init_pose.y << std::endl;
      init_pose.phi = yaw;
      gml->UpdatePointCloudPose(init_pose);

    } break;
  }

  server->applyChanges();
}
// slam区域的绘制函数
void SlamAreaPublishingCallback(const ros::TimerEvent &) {
  visualization_msgs::MarkerArray marker_array;
  for (int i = 0; i < gml->vec_Slam_Area_cp_.size(); i++) {
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "points_and_lines";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = i;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 1;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    geometry_msgs::Point p;
    for (int j = 0; j < gml->vec_Slam_Area_cp_[i].size(); j++) {
      p.x = gml->vec_Slam_Area_cp_[i][j].x;
      p.y = gml->vec_Slam_Area_cp_[i][j].y;
      line_strip.points.push_back(p);
    }
    p.x = gml->vec_Slam_Area_cp_[i][0].x;
    p.y = gml->vec_Slam_Area_cp_[i][0].y;
    line_strip.points.push_back(p);
    marker_array.markers.push_back(line_strip);
  }
  marker_publisher->publish(marker_array);
}

//拟合轨迹直线推断航向
std::vector<float> LineFitLeastSquares(
    std::list<localization::Pose2D> pose_list, int data_n) {
  float A = 0.0;
  float B = 0.0;
  float C = 0.0;
  float D = 0.0;
  float E = 0.0;
  float F = 0.0;
  std::vector<float> vResult;

  std::list<localization::Pose2D>::iterator p;
  for (p = pose_list.begin(); p != pose_list.end(); ++p) {
    double data_x = p->x;
    double data_y = p->y;
    A += data_x * data_x;
    B += data_x;
    C += data_x * data_y;
    D += data_y;
  }

  // 计算斜率a和截距b
  float a, b, temp = 0;
  if (temp = (data_n * A - B * B))  // 判断分母不为0
  {
    a = (data_n * C - B * D) / temp;
    b = (A * D - B * C) / temp;
  } else {
    a = 1;
    b = 0;
  }

  // 计算相关系数r
  float Xmean, Ymean;
  Xmean = B / data_n;
  Ymean = D / data_n;

  float tempSumXX = 0.0, tempSumYY = 0.0;
  for (p = pose_list.begin(); p != pose_list.end(); ++p) {
    double data_x = p->x;
    double data_y = p->y;
    tempSumXX += (data_x - Xmean) * (data_x - Xmean);
    tempSumYY += (data_y - Ymean) * (data_y - Ymean);
    E += (data_x - Xmean) * (data_y - Ymean);
  }
  F = sqrt(tempSumXX) * sqrt(tempSumYY);

  float r;
  r = E / F;

  vResult.push_back(a);
  vResult.push_back(b);
  vResult.push_back(r * r);

  return vResult;
}
