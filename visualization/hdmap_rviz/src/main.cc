/**************************************************
 *  Created by ShenQiyue and WangLiang on 19/11/4
**************************************************/
#include "ros/ros.h"
#include "string.h"
#include "map_rviz.h"

// USB Key，只在生成发布版的时候才解注释
// #include "../../../common/Pwd_8/SoftkeyPWD.h"

int main(int argc, char **argv)
{
    // 只在生成发布版的时候才解注释，否则在自己电脑上无法调试
    // if(!checkUSBKey()) return 0;
    
    //Initialize ROS node
    ros::init(argc, argv, "map_rviz_node");
    ros::NodeHandle pnh("~");
    visualization_msgs::MarkerArray marker_arrary;

    //Reading parameters
    bool publish_stop_line;
    bool publish_boundary;
    bool publish_center_line;
    bool publish_junction_line;
    bool default_publish_stop_line = true;
    bool default_publish_boundary = true;
    bool default_publish_center_line = true;
    bool default_publish_junction_line = true;
    std::string route;
    std::string map_storage_path;
    std::string map_rviz_frame;
    std::string map_data_json;
    std::string boundary_map_path;
    double publish_frequency;
    std::string default_route = "BJ12";
    std::string default_map_storage_path =
        expand_catkin_ws("/src/map/hdmap/data/");
    std::string default_boundary_map_path = "../src/CyberMars/map/hdmap/data/boundary_space.txt";
    std::string default_map_rviz_frame = "world";
    std::string default_map_data_json = "../src/CyberMars/map/hdmap/data/SJTU.json";
    double default_publish_frequency = 0.1;
    pnh.param("route", route, default_route);
    pnh.param<std::string>("map_storage_path", map_storage_path, default_map_storage_path);
    pnh.param<std::string>("frame", map_rviz_frame, default_map_rviz_frame);
    pnh.param<double>("publish_frequency", publish_frequency, default_publish_frequency);
    pnh.param<bool>("publish_stop_line", publish_stop_line, default_publish_stop_line);
    pnh.param<bool>("publish_boundary", publish_boundary, default_publish_boundary);
    pnh.param<bool>("publish_center_line", publish_center_line, default_publish_center_line);
    pnh.param<bool>("publish_junction_line", publish_junction_line, default_publish_junction_line);

    map_data_json = map_storage_path + route + "/" + route + ".json";
    boundary_map_path = map_storage_path + route + "/" + "boundary.txt";

    double utm_origin_x, utm_origin_y;
    double default_utm_origin_x = 355000;
    double default_utm_origin_y = 2700000;
    pnh.param("GLOBAL_ZERO_X_", utm_origin_x, default_utm_origin_x);
    pnh.param("GLOBAL_ZERO_Y_", utm_origin_y, default_utm_origin_y);

    Map_Rviz map_rviz(map_data_json, boundary_map_path, map_rviz_frame, utm_origin_x, utm_origin_y);

    marker_arrary = map_rviz.GetMarkerArray();
    //Publish
    ros::Publisher map_pub = pnh.advertise<visualization_msgs::MarkerArray>("map_marker", 10);
    ros::Rate r(publish_frequency);
    while (ros::ok())
    {
        map_pub.publish(marker_arrary);
        r.sleep();
    }
    return 0;
}