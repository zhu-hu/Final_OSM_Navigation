#ifndef MAP_RVIZ_H
#define MAP_RVIZ_H

#include "ros/ros.h"
#include "map/hdmap/src/Impl.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "string.h"
#include <fstream>

using Marker = visualization_msgs::Marker;
using MarkerArray = visualization_msgs::MarkerArray;

struct Point
{
    double x;
    double y;
};

class Map_Rviz
{
public:
    //Construct funciton
    Map_Rviz(const std::string &input_map_path,
             const std::string &input_boundary_map,
             const std::string &input_publish_frame,
             const double& utm_origin_x,
             const double& utm_origin_y);
    ~Map_Rviz();

private:
    //Private member
    std::string input_map_path;
    std::string boundary_file_path;
    std::string publish_frame;
    hdmap::HDMapImpl *map_data = new hdmap::HDMapImpl();
    bool ready_to_generate = false;
    double utm_origin_x_;
    double utm_origin_y_;

    //Declearation of LaneInfoTable
    hdmap::HDMapImpl::LaneTable lane_table;
    hdmap::HDMapImpl::JunctionTable junction_table;
    hdmap::HDMapImpl::SignalTable signal_table;
    hdmap::HDMapImpl::StopSignTable stopsign_table;
    hdmap::HDMapImpl::OverlapTable overlap_table;

    //Declearation of table points
    std::vector<std::vector<Point>> junction_lanes_teble;
    std::vector<std::vector<Point>> normal_lanes_teble;
    std::vector<std::vector<Point>> stop_sign_table;
    std::vector<std::vector<Point>> junciton_points_table;
    std::vector<std::string> lane_id_table;

    //Declearation of markers
    MarkerArray marker_arrary;
    MarkerArray boundary_marker_arrary;
    MarkerArray junciton_marker_arrary;
    MarkerArray center_line_marker_arrary;
    MarkerArray junction_line_marker_arrary;
    MarkerArray stop_sign_marker_arrary;
    MarkerArray lane_line_marker_arrary;
    MarkerArray lane_id_marker_array;

    //Choose which to publish
    bool publish_boundary = true;
    bool publish_center_line = true;
    bool publish_junction_line = true;
    bool publish_stop_sign = true;
    bool publish_juncction = true;

    //Private fucntion
    void draw_boundary_marker(const std::vector<geometry_msgs::Point> &input_boundary_points, const int &id_count);
    void draw_junction_marker(const std::vector<Point> &input_junciton_points, const int &id_count);
    void draw_junction_line_marker(const std::vector<Point> &input_junction_line_points, const int &id_count);
    void draw_center_line_marker(const std::vector<Point> &input_center_line_points, const int &id_count);
    void draw_stop_sign_marker(const std::vector<Point> &input_stop_sign_points, const int &id_count);
    void draw_lane_line_marker(const std::vector<Point> &input_lane_line_points, const int &id_count);
    void draw_lane_id_marker(const hdmap::HDMapImpl::LaneTable &lane_table);

    void GenerateMarkerArray();

public:
    MarkerArray GetMarkerArray();
    void SetChoices(bool publish_boundary_,
                    bool publish_center_line_,
                    bool publish_junction_line_,
                    bool publish_stop_sign_, bool publish_junction_);
    void ResetPublishFrame(const std::string &input_publish_frame);
};

#endif