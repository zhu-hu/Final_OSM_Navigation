#include <memory>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "dstar_map/WayPoint.h"
#include "dstar_map/WayPointList.h"
#include "cyber_msgs/LocalTrajList.h"
#include "cyber_msgs/LocalTrajPoint.h"
#include "rrt/rrt.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt");
    ros::NodeHandle pnh("~");

    tf::TransformListener *listener = new tf::TransformListener();

    huyao_rrt::RRT my_rrt(&pnh, listener, 0.05, 5.0);

    ros::spin();

    return 0;
}
