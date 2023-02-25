//
// Created by luyifan on 18-8-20.
//

#ifndef CREATE_MAP_CREATEMAP_H
#define CREATE_MAP_CREATEMAP_H

#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>

#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "dstar_map/WayPoint.h"
#include "cyber_msgs/LocalTrajList.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "image_transport/image_transport.h"
#include "nav_msgs/OccupancyGrid.h"
#include "dstar_map/MapUpdate.h"
#include "dstar_map/CellUpdate.h"
#include "dstar_map/CellUpdateList.h"
#include "dstar_map/RoadCheckGoal.h"
#include "cyber_msgs/LocalizationEstimate.h"

#include "ros_transform.h"
#include "log.h"
#include "map_param.h"

typedef std::pair<std::string, ros::Publisher> PubMapPair;

typedef struct PixPoint{
    int x;
    int y;
} PixPoint;

typedef boost::shared_lock<boost::shared_mutex> read_lock;
typedef boost::unique_lock<boost::shared_mutex> write_lock;

constexpr double kCellSize = 0.2;

class CreateMap{
public:
    CreateMap(ros::NodeHandle* nh, tf::TransformListener* listener, tf::TransformBroadcaster* broadcaster);
    ~CreateMap();

    void InitMap(dstar_map::WayPoint start, dstar_map::WayPoint goal, cv::Mat &map_in);
    void UpdateMap(const sensor_msgs::CompressedImageConstPtr &map_in);
    void PubOccMap(const cv::Mat* map_in, std::string frame_id);
    void DeleteMap();
    void SetTaskRviz(const geometry_msgs::PoseStampedConstPtr& click);
    bool TaskGoalCallback(dstar_map::RoadCheckGoal::Request& req, dstar_map::RoadCheckGoal::Response &res);
    void SetTaskStateMachine(const geometry_msgs::PoseStampedConstPtr& goal);
    void CpCallback(const cyber_msgs::LocalizationEstimate& pose);
    void Timer1Callback(const ros::TimerEvent&);
private:
    ros::NodeHandle* nh_;
    ///任务的起点和终点,单位是像素
    //PixPoint start_;
    //PixPoint goal_;
    dstar_map::WayPoint start_;
    dstar_map::WayPoint goal_;
    int task_id_;

    ///全局地图相关
    cv::Mat* global_map_;
    geometry_msgs::Pose global_map_origin_;
    bool bMapInit_;
    bool bTFInit_;
    bool bDStarInit_;
    bool bWaitInit_;
    int T_update_;

    ///局部地图相关
    cv::Mat local_map_;
    cv::Mat kernelDilateL_;

    geometry_msgs::PoseStamped cp_;

    std::vector<PubMapPair> pub_occ_map_;
    image_transport::ImageTransport* it_;
    ros::Subscriber sub_local_map_;
    ros::Subscriber sub_task_rviz_;
    ros::Subscriber sub_state_machine_task_;
    ros::Subscriber sub_cp_;
    ros::Publisher task_goal_pub_;
    tf::TransformListener* listener_;
    tf::TransformBroadcaster* broadcaster_;
    ros::ServiceClient client_task_request_;
    ros::Timer timer1_;

    ros::CallbackQueue queue_1_;
    ros::CallbackQueue queue_2_;

    ros::AsyncSpinner* spinner_1_;
    ros::AsyncSpinner* spinner_2_;

    boost::shared_mutex global_map_mutex_;///GlobalMap的读写锁
    boost::shared_mutex local_map_mutex_;///LocalMap的读写锁
};

#endif //CREATE_MAP_CREATEMAP_H
