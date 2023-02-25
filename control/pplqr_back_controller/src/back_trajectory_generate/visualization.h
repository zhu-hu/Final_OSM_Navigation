#pragma once

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <opencv/cv.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <list>
#include <memory>
#include <vector>

#include "common/log.h"
#include "common/params/vehicle_param.h"
#include "common/struct/pnc_point.h"
#include "cyber_msgs/LinkAngle.h"
#include "cyber_msgs/LocalTrajList.h"
#include "cyber_msgs/LocalizationEstimate.h"

namespace visualization {
void PublishGlobalTrajectory(
    const ros::Publisher &pub,
    const std::list<cyber_msgs::LocalTrajList> &trajectories);
void PubLocalMap(const ros::Publisher &pub, const cv::Mat &map_in,
                 const double &originX, const double &originY);
void PublishRealPath(const ros::Publisher &pub, const nav_msgs::Path &real_path,
                     const std::string color);
void ShowVehiclePose(const ros::Publisher &pub,
                     const cyber_msgs::LocalizationEstimateConstPtr &pose_in);
void ShowVehiclePose(const ros::Publisher &pub,
                     const cyber_msgs::LocalizationEstimateConstPtr &pose_in,
                     const cyber_msgs::LinkAngleConstPtr &link_angle_in);
}  // namespace visualization
