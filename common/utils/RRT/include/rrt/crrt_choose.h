//
// Created by huyao on 18-8-24.
//

#ifndef PROJECT_RRT_CHOOSE_H
#define PROJECT_RRT_CHOOSE_H


#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Path.h>
#include "cyber_msgs/LocalTrajList.h"
#include "cyber_msgs/LocalTrajPoint.h"
#include "dstar_map/WayPointList.h"
#include "dstar_map/WayPoint.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/Path.h"
#include "vec2d.h"
#include "ros_transform.h"
#include "log.h"
#include "vec2d.h"
#include "math_utils.h"
#include "ros_transform.h"
#include <tf/transform_listener.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

namespace huyao_RRT{

    class CRRTChoose {
    public:
        CRRTChoose(ros::NodeHandle* nh, tf::TransformListener *listener);
        virtual ~CRRTChoose();

    private:
        void GridMapCallback(const sensor_msgs::ImageConstPtr &map_in);

    public:
        void publishBestPath();

        void path1Callback(const nav_msgs::PathConstPtr &path_in);
        void path2Callback(const nav_msgs::PathConstPtr &path_in);
        void path3Callback(const nav_msgs::PathConstPtr &path_in);
        void path4Callback(const nav_msgs::PathConstPtr &path_in);
        void path5Callback(const nav_msgs::PathConstPtr &path_in);
        void path6Callback(const nav_msgs::PathConstPtr &path_in);
        void StartCallback(const geometry_msgs::PoseStampedPtr &start);

        bool run();


    private:
        ros::NodeHandle* nh_;
        ros::Publisher trajectory_pub;
        ros::Publisher local_path_pub;

        ros::Subscriber path1Sub_;
        ros::Subscriber path2Sub_;
        ros::Subscriber path3Sub_;
        ros::Subscriber path4Sub_;
        ros::Subscriber path5Sub_;
        ros::Subscriber path6Sub_;
        ros::Subscriber subStart_;

        std::map<double,nav_msgs::Path::ConstPtr>pathMap_;

        cyber_msgs::LocalTrajList trajectory_;//local path generated is stored here.
        tf::TransformListener *listener_;

        ros::Time publishTime_;
        bool bPublish_;

    };

}//namespace huyao_RRT


#endif //PROJECT_RRT_CHOOSE_H
