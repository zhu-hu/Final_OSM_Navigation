#ifndef DSTAR_DSTAR_H
#define DSTAR_DSTAR_H

#include <sbpl/headers.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv/cv.h>
#include "geometry_msgs/PointStamped.h"
#include "dstar_map/WayPoint.h"
#include "cyber_msgs/LocalTrajList.h"
#include "nav_msgs/Path.h"

#include "ros_transform.h"
#include "log.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "dstar_map/CellUpdateList.h"
#include "dstar_map/CellUpdate.h"
#include "dstar_map/MapUpdate.h"

constexpr double kCellSize = 0.2;

class Dstar {
public:
    Dstar(ros::NodeHandle* nh,
          int obs_thresh,
          double runtime,
          ros::Publisher* pub_path,
          ros::Publisher* pub_target);
    ~Dstar() {};

public:
    void InitMap(geometry_msgs::PoseStamped cp_, int goal_x, int goal_y, float goal_theta, int length, dstar_map::CellUpdateList update_list);
    void UpdateMap(dstar_map::CellUpdateList update_list);
    void PubPath(cyber_msgs::LocalTrajList* path);
    void DeletePreTask();
    void PubLocalTarget(geometry_msgs::PoseStamped cp_, cyber_msgs::LocalTrajList* path);

    bool RePlan(geometry_msgs::PoseStamped cp_, cyber_msgs::LocalTrajList* path);
    bool RePlan(geometry_msgs::PoseStamped cp_, cyber_msgs::LocalTrajList* path, double time);

private:
    ///D*参数
    EnvironmentNAV2D environmentNav2D_;
    unsigned char *map_;
    MDPConfig mDPCfg_;
    SBPLPlanner *planner_;
    double allocated_time_secs_foreachplan_;
    int obs_thresh_;
    double goal_theta_;

    ///地图相关参数
    int width_;
    int height_;
    boost::shared_mutex read_write_mutex_;///GlobalMap的读写锁

    ///ROS
    ros::NodeHandle* nh_;
    tf::TransformListener* listener_;
    ros::Publisher* pub_path_;
    ros::Publisher* pub_target_;

    geometry_msgs::Quaternion goal_quaternion_;
};


#endif //DSTAR_DSTAR_H
