#include "map_rviz.h"

Map_Rviz::Map_Rviz(const std::string &input_map_path,
                   const std::string &input_boundary_map,
                   const std::string &input_publish_frame,
                   const double& utm_origin_x,
                   const double& utm_origin_y)
{
    utm_origin_x_ = utm_origin_x;
    utm_origin_y_ = utm_origin_y;
    if (map_data->LoadMap(input_map_path, utm_origin_x_, utm_origin_y_) == -1)
    {
        ROS_INFO("Failed to load map!");
        ready_to_generate = false;
    }
    else if (input_publish_frame.size() == 0)
    {
        ready_to_generate = false;
    }
    else
    {
        publish_frame = input_publish_frame;
        boundary_file_path = input_boundary_map;
        ready_to_generate = true;
    }
}
Map_Rviz::~Map_Rviz()
{
    delete map_data;
}
void Map_Rviz::draw_boundary_marker(const std::vector<geometry_msgs::Point> &input_boundary_points, const int &id_count)
{
    visualization_msgs::Marker boundary_marker;
    boundary_marker.header.frame_id = publish_frame;
    boundary_marker.header.seq = id_count;
    boundary_marker.header.stamp = ros::Time::now();
    boundary_marker.ns = "boundary_points";
    boundary_marker.action = visualization_msgs::Marker::ADD;
    boundary_marker.type = visualization_msgs::Marker::LINE_STRIP;
    boundary_marker.id = id_count;
    boundary_marker.lifetime = ros::Duration(0);
    //Scale
    boundary_marker.scale.x = 0.7;
    boundary_marker.scale.y = 0.7;
    boundary_marker.scale.z = 0.7;
    //Color
    boundary_marker.color.r = 0.99;
    boundary_marker.color.g = 0.99;
    boundary_marker.color.b = 1.0;
    boundary_marker.color.a = 1;
    //Localizaion
    for (auto i : input_boundary_points)
    {
        geometry_msgs::Point temp_point;
        temp_point.x = i.x;
        temp_point.y = i.y;
        temp_point.z = 0.1;
        boundary_marker.points.push_back(temp_point);
    }
    boundary_marker_arrary.markers.push_back(boundary_marker);
    marker_arrary.markers.push_back(boundary_marker);
}
void Map_Rviz::draw_junction_marker(const std::vector<Point> &input_junciton_points, const int &id_count)
{
    visualization_msgs::Marker junction_marker;
    junction_marker.header.frame_id = publish_frame;
    junction_marker.header.seq = id_count;
    junction_marker.header.stamp = ros::Time::now();
    junction_marker.ns = "junciton_points";
    junction_marker.action = visualization_msgs::Marker::ADD;
    junction_marker.type = visualization_msgs::Marker::LINE_STRIP;
    junction_marker.id = id_count;
    junction_marker.lifetime = ros::Duration(0);
    //Scale
    junction_marker.scale.x = 0.3;
    junction_marker.scale.y = 0.3;
    junction_marker.scale.z = 0.3;
    //Color
    junction_marker.color.r = 0;
    junction_marker.color.g = 0.99;
    junction_marker.color.b = 0.99;
    junction_marker.color.a = 0.3;
    //Localizaion
    for (auto i : input_junciton_points)
    {
        geometry_msgs::Point temp_point;
        temp_point.x = i.x;
        temp_point.y = i.y;
        temp_point.z = 0.1;
        junction_marker.points.push_back(temp_point);
    }
    junciton_marker_arrary.markers.push_back(junction_marker);
    marker_arrary.markers.push_back(junction_marker);
}
void Map_Rviz::draw_junction_line_marker(const std::vector<Point> &input_junction_line_points, const int &id_count)
{
    visualization_msgs::Marker junction_line_marker;
    junction_line_marker.header.frame_id = publish_frame;
    junction_line_marker.header.seq = id_count;
    junction_line_marker.header.stamp = ros::Time::now();
    junction_line_marker.ns = "junciton_line_points";
    junction_line_marker.action = visualization_msgs::Marker::ADD;
    junction_line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    junction_line_marker.id = id_count;
    junction_line_marker.lifetime = ros::Duration(0);
    //Scale
    junction_line_marker.scale.x = 0.3;
    junction_line_marker.scale.y = 0.3;
    junction_line_marker.scale.z = 0.3;
    //Color
    junction_line_marker.color.r = 1;
    junction_line_marker.color.g = 0.546;
    junction_line_marker.color.b = 0.0;
    junction_line_marker.color.a = 0.5;
    //Localizaion
    for (auto i : input_junction_line_points)
    {
        geometry_msgs::Point temp_point;
        temp_point.x = i.x;
        temp_point.y = i.y;
        temp_point.z = 0.1;
        junction_line_marker.points.push_back(temp_point);
    }
    junction_line_marker_arrary.markers.push_back(junction_line_marker);
    marker_arrary.markers.push_back(junction_line_marker);
}

void Map_Rviz::draw_center_line_marker(const std::vector<Point> &input_center_line_points, const int &id_count)
{
    visualization_msgs::Marker center_line_marker;
    center_line_marker.header.frame_id = publish_frame;
    center_line_marker.header.seq = id_count;
    center_line_marker.header.stamp = ros::Time::now();
    center_line_marker.ns = "center_line_points";
    center_line_marker.action = visualization_msgs::Marker::ADD;
    center_line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    center_line_marker.id = id_count;
    center_line_marker.lifetime = ros::Duration(0);
    //Scale
    center_line_marker.scale.x = 0.3;
    center_line_marker.scale.y = 0.3;
    center_line_marker.scale.z = 0.3;
    //Color
    center_line_marker.color.r = 0.8823;
    center_line_marker.color.g = 1;
    center_line_marker.color.b = 1;
    center_line_marker.color.a = 0.5;
    //Localizaion
    for (auto i : input_center_line_points)
    {
        geometry_msgs::Point temp_point;
        temp_point.x = i.x;
        temp_point.y = i.y;
        temp_point.z = 0.1;
        center_line_marker.points.push_back(temp_point);
    }
    center_line_marker_arrary.markers.push_back(center_line_marker);
    marker_arrary.markers.push_back(center_line_marker);
}

void Map_Rviz::draw_stop_sign_marker(const std::vector<Point> &input_stop_sign_points, const int &id_count)
{
    visualization_msgs::Marker stop_sign_marker;
    stop_sign_marker.header.frame_id = publish_frame;
    stop_sign_marker.header.seq = id_count;
    stop_sign_marker.header.stamp = ros::Time::now();
    stop_sign_marker.ns = "stop_sign_points";
    stop_sign_marker.action = visualization_msgs::Marker::ADD;
    stop_sign_marker.type = visualization_msgs::Marker::LINE_STRIP;
    stop_sign_marker.id = id_count;
    stop_sign_marker.lifetime = ros::Duration(0);
    //Scale
    stop_sign_marker.scale.x = 0.3;
    stop_sign_marker.scale.y = 0.3;
    stop_sign_marker.scale.z = 0.3;
    //Color
    stop_sign_marker.color.r = 1;
    stop_sign_marker.color.g = 0.3;
    stop_sign_marker.color.b = 0.3;
    stop_sign_marker.color.a = 0.3;
    //Localizaion
    for (auto i : input_stop_sign_points)
    {
        geometry_msgs::Point temp_point;
        temp_point.x = i.x;
        temp_point.y = i.y;
        temp_point.z = 0.1;
        stop_sign_marker.points.push_back(temp_point);
    }
    stop_sign_marker_arrary.markers.push_back(stop_sign_marker);
    marker_arrary.markers.push_back(stop_sign_marker);
}

void Map_Rviz::draw_lane_line_marker(const std::vector<Point> &input_lane_line_points, const int &id_count)
{
}

void Map_Rviz::draw_lane_id_marker(const hdmap::HDMapImpl::LaneTable &lane_table)
{
    int id_count = 0;
    for (auto lane : lane_table)
    {
        visualization_msgs::Marker lane_id_marker;
        lane_id_marker.header.frame_id = publish_frame;
        lane_id_marker.header.seq = id_count;
        lane_id_marker.header.stamp = ros::Time::now();
        lane_id_marker.ns = "lane_id_marker";
        lane_id_marker.action = visualization_msgs::Marker::ADD;
        lane_id_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        lane_id_marker.id = id_count;
        lane_id_marker.lifetime = ros::Duration(0);
        std::string s = " sucessors: ";
        s = lane.second->id() + s;
        for(auto sucessor : lane.second->lane().successor_ids){
            s += *sucessor + " ";
        }

        lane_id_marker.text = s;
        //Scale
        lane_id_marker.scale.x = 1;
        lane_id_marker.scale.y = 1;
        lane_id_marker.scale.z = 1;
        //Color
        lane_id_marker.color.r = 0.5;
        lane_id_marker.color.g = 0.5;
        lane_id_marker.color.b = 0.7;
        lane_id_marker.color.a = 1;
        //Localizaion
        int size_point = lane.second->points().size();
        if (size_point % 2 == 0)
            size_point /= 2;
        else
            size_point = (size_point + 1) / 2;

        lane_id_marker.pose.position.x = lane.second->points()[size_point].x();
        lane_id_marker.pose.position.y = lane.second->points()[size_point].y();
        lane_id_marker.pose.position.z = 0;
        // AINFO << lane_id_marker.text << "|" << lane_id_marker.pose.position.x << "|" << lane_id_marker.pose.position.y;
        lane_id_marker_array.markers.push_back(lane_id_marker);
        marker_arrary.markers.push_back(lane_id_marker);
        ++id_count;
    }
}

void Map_Rviz::GenerateMarkerArray()
{
    if (!ready_to_generate)
    {
        ROS_INFO("Please set the parameters correctly!");
        return;
    }
    //Get tables from map_data
    lane_table = map_data->GetLaneTable();
    signal_table = map_data->GetSignalTable();
    stopsign_table = map_data->GetStopSignTable();
    overlap_table = map_data->GetOverlapTable();

    //Get boundary points
    std::vector<geometry_msgs::Point> line_data;
    std::vector<std::vector<geometry_msgs::Point>> boundary_points_data;
    std::ifstream boundary_txt(boundary_file_path);
    std::string txt_line;
    std::string longt;
    std::string latit;
    double temp_longt;
    double temp_latit;
    double temp_x;
    double temp_y;
    int segement_count = 0;
    if (boundary_txt)
    {
        while (getline(boundary_txt, txt_line))
        {
            if (txt_line.size() <= 1)
            {
                if (line_data.size() > 1)
                {
                    boundary_points_data.push_back(line_data);
                    line_data.clear();
                }
                continue;
            }
            std::stringstream input_long_lati(txt_line);
            input_long_lati >> longt >> latit;
            temp_longt = std::stod(longt);
            temp_latit = std::stod(latit);
            char zone = 0;
            LLtoUTM(temp_latit, temp_longt, temp_y, temp_x, &zone, utm_origin_x_, utm_origin_y_);
            geometry_msgs::Point temp_point;
            temp_point.x = temp_x;
            temp_point.y = temp_y;
            temp_point.z = 0;
            line_data.push_back(temp_point);
        }
    }

    //Get Lane Points
    for (auto lane : lane_table)
    {
        if (lane.second->id()[0] == 'j') //Jucntion
        {
            std::vector<Point> temp_junction_lane;
            for (auto junction_point : lane.second->points())
            {
                Point temp_junction_lane_point;
                temp_junction_lane_point.x = junction_point.x();
                temp_junction_lane_point.y = junction_point.y();
                temp_junction_lane.push_back(temp_junction_lane_point);
            }
            junction_lanes_teble.push_back(temp_junction_lane);
        }
        else
        {
            //Not Junction
            //Get center_line points
            std::vector<Point> temp_normal_lane;
            for (auto center_lane_point : lane.second->points())
            {
                Point temp_normal_lane_point;
                temp_normal_lane_point.x = center_lane_point.x();
                temp_normal_lane_point.y = center_lane_point.y();
                temp_normal_lane.push_back(temp_normal_lane_point);
            }
            normal_lanes_teble.push_back(temp_normal_lane);
        }
    }

    //Get Stop Sign
    for (auto stop_sign : stopsign_table)
    {
        std::vector<Point> temp_stop_sign;
        for (auto stop_sign_segement : stop_sign.second->segments())
        {
            Point temp_stop_sign_start, temp_stop_sign_end;
            temp_stop_sign_start.x = stop_sign_segement.start().x();
            temp_stop_sign_start.y = stop_sign_segement.start().y();
            temp_stop_sign_end.x = stop_sign_segement.end().x();
            temp_stop_sign_end.y = stop_sign_segement.end().y();
            temp_stop_sign.push_back(temp_stop_sign_start);
            temp_stop_sign.push_back(temp_stop_sign_end);
        }
        stop_sign_table.push_back(temp_stop_sign);
    }

    //Get Junction
    for (auto junction : junction_table)
    {
        std::vector<Point> temp_junction_points;
        for (auto junction_point : junction.second->polygon().points())
        {
            Point temp_junction_point;
            temp_junction_point.x = junction_point.x();
            temp_junction_point.y = junction_point.y();
            temp_junction_points.push_back(temp_junction_point);
        }
        junciton_points_table.push_back(temp_junction_points);
    }
    if (publish_boundary)
    {
        int boundary_count = 0;
        for (auto boundary_point : boundary_points_data)
        {
            draw_boundary_marker(boundary_point, boundary_count);
            boundary_count++;
        }
    }

    if (publish_junction_line)
    {
        int junction_lane_count = 0;
        for (auto junction_lane : junction_lanes_teble)
        {
            draw_junction_line_marker(junction_lane, junction_lane_count);
            ++junction_lane_count;
        }
    }

    if (publish_center_line)
    {
        int lane_count = 0;
        for (auto lane : normal_lanes_teble)
        {
            draw_center_line_marker(lane, lane_count);
            ++lane_count;
        }
    }

    if (publish_stop_sign)
    {
        int stop_sign_count = 0;
        for (auto stop_sign : stop_sign_table)
        {
            draw_stop_sign_marker(stop_sign, stop_sign_count);
            stop_sign_count++;
        }
    }
    if (publish_juncction)
    {
        int junction_count = 0;
        for (auto junction : junciton_points_table)
        {
            draw_junction_marker(junction, junction_count);
            ++junction_count;
        }
    }
    draw_lane_id_marker(lane_table);
}
void Map_Rviz::SetChoices(bool publish_boundary_,
                          bool publish_center_line_,
                          bool publish_junction_line_,
                          bool publish_stop_sign_, bool publish_junction_)
{
    publish_boundary = publish_boundary_;
    publish_center_line = publish_center_line_;
    publish_junction_line = publish_junction_line_;
    publish_stop_sign = publish_stop_sign_;
    publish_juncction = publish_junction_;
}
MarkerArray Map_Rviz::GetMarkerArray()
{
    GenerateMarkerArray();
    return marker_arrary;
}