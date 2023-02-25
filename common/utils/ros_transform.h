#ifndef DSTAR_TOOL_H
#define DSTAR_TOOL_H

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_listener.h"
#include "string.h"
#include "../utils/DSTAR/dstar_map/include/create_map/map_param.h"

typedef boost::shared_lock<boost::shared_mutex> read_lock;
typedef boost::unique_lock<boost::shared_mutex> write_lock;

inline bool TransformPoint(tf::TransformListener *listener,
                           const std::string source_frame,
                           const std::string desire_frame,
                           geometry_msgs::PointStamped &source_pt,
                           geometry_msgs::PointStamped &desire_pt)
{

    source_pt.header.frame_id = source_frame;
    try
    {
        listener->transformPoint(desire_frame, source_pt, desire_pt);
        return true; ///转换成功
    }
    catch (tf::TransformException ex)
    {
        //std::cout<<"Test2.2"<<std::endl;
        // ROS_ERROR("%s", ex.what());
        // ROS_ERROR("TF error1!");
        ros::Duration(1.0).sleep();
        return false; ///转换失败
    }
}

inline bool TransformPose(tf::TransformListener *listener,
                          const std::string source_frame,
                          const std::string desire_frame,
                          geometry_msgs::PoseStamped &source_pose,
                          geometry_msgs::PoseStamped &desire_pose)
{
    source_pose.header.frame_id = source_frame;
    try
    {
        listener->transformPose(desire_frame, source_pose, desire_pose);
        return true; ///转换成功
    }
    catch (tf::TransformException ex)
    {
        // ROS_ERROR("%s", ex.what());
        // ROS_ERROR("TF error1!");
        ros::Duration(1.0).sleep();
        return false; ///转换失败
    }
}

///将local_map像素单位转换为base_link的米制单位
inline void TFPix2XYInCar(int pix_x, int pix_y, double &x, double &y)
{
    x = -(pix_y - map_param::grid_map::kCarCenterYCVIndex) * map_param::grid_map::kCellSize;
    y = -(pix_x - map_param::grid_map::kCarCenterX) * map_param::grid_map::kCellSize;
}

///将local_map像素单位转换为base_link的米制单位
inline void TFPix2XYInCar(double pix_x, double pix_y, double &x, double &y)
{
    x = -(pix_y - map_param::grid_map::kCarCenterYCVIndex) * map_param::grid_map::kCellSize;
    y = -(pix_x - map_param::grid_map::kCarCenterX - 0.5) * map_param::grid_map::kCellSize;
}

///将local_mapbase_link的米制单位转换为像素单位
inline void TFXY2PixInCar(double x, double y, double &pix_x, double &pix_y)
{
    pix_x = map_param::grid_map::kCarCenterX + 0.5 - y * map_param::grid_map::kCellFactor;
    pix_y = map_param::grid_map::kCarCenterYCVIndex - x * map_param::grid_map::kCellFactor;
}

///将local_mapbase_link的米制单位转换为像素单位
inline void TFXY2PixInCar(double x, double y, int &pix_x, int &pix_y)
{
    pix_x = map_param::grid_map::kCarCenterX + 0.5 - y * map_param::grid_map::kCellFactor;
    pix_y = map_param::grid_map::kCarCenterYCVIndex - x * map_param::grid_map::kCellFactor;
}

///输入:
/// x->米制x
/// y->米制y
/// origin_x->原点像素x
/// origin_y->原点像素y
/// -----------------------
///输出:
/// pix_x->像素x
/// pix_y->像素y
inline void TFXY2PixInGlobalMap(const double x, const double y,
                                int &pix_x, int &pix_y,
                                const int rows)
{
    pix_x = map_param::grid_map::kCellFactor * x;
    pix_y = rows - map_param::grid_map::kCellFactor * y;
}

inline void TFPix2XYInGlobalMap(double &x, double &y,
                                const int pix_x, const int pix_y,
                                const int rows)
{
    x = pix_x * map_param::grid_map::kCellSize;
    y = (rows - pix_y) * map_param::grid_map::kCellSize;
}
#endif //DSTAR_TOOL_H
