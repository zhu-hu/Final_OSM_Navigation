//
// Created by cyber on 18-10-19.
//

#ifndef RRT_TRAJECTORYCOMBINER_H
#define RRT_TRAJECTORYCOMBINER_H

#include "tf/transform_listener.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "ros/ros.h"
#include "cyber_msgs/LocalTrajList.h"
#include "cyber_msgs/LocalTrajPoint.h"
#include "log.h"
#include "ros_transform.h"

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "nav_msgs/Path.h"
#include "rrt/rrt.h"
#include "cyber_msgs/LocalizationEstimate.h"

#define kCutDistance (10.0) ///保留路径的长度，后面切掉
#define kMaxSpeed (3.0)
#define kRemainDis (6.0)

class TrajectoryCombine{
public:
    TrajectoryCombine(ros::NodeHandle* nh, tf::TransformListener* listener);
    ~TrajectoryCombine();

    void CompleteRRTCallback(const cyber_msgs::LocalTrajList::ConstPtr& traj_in);
    void InCompleteRRTCallback(const cyber_msgs::LocalTrajList::ConstPtr& traj_in);
    void GridMapCallback(const sensor_msgs::ImageConstPtr &map_in);
    bool CheckTrajGrad(float &speed);
    void PubCRRTStart();///将当前路径终点发布给RRT规划节点，生成新的路径用于拼接
    bool CheckTrajFree(const cyber_msgs::LocalTrajList& traj_in);
    void CurrentPoseCallback(const cyber_msgs::LocalizationEstimate& cp);
    void TaskGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal);
    bool CutTrajectory(cyber_msgs::LocalTrajList& traj_in);

private:
    ros::NodeHandlePtr nh_;
    ros::Subscriber subCompleteRRT_;///订阅从当前位置到终点的RRT路径
    ros::Subscriber subGridMap_;
    ros::Subscriber subIncompleteRRT_;
    ros::Subscriber subCurrentPose_;
    ros::Subscriber subTaskGoal_;
    ros::Publisher pubRRT_;
    ros::Publisher pubRRTRviz_;
    ros::Publisher pubStartSecTraj_;//、下一段RRT路径的起点

    tf::TransformListener* listener_;

    cyber_msgs::LocalTrajList trajectory_;//当前正在使用的路径
    cyber_msgs::LocalTrajList newTrajectory_;///下一周期将使用的
    cyber_msgs::LocalTrajList firstTrajectory_;///被切的第一段路径
    cyber_msgs::LocalTrajList secTrajectory_;///用于拼接的第二段路径

    geometry_msgs::PoseStamped currentPose_;
    geometry_msgs::PoseStamped currentGoal_;
    cv::Mat localMap_;

    float maxSpeed_;

    bool bSecTrajIsValid_;
    bool bNeedCut_;
    bool bFirstTrajIsFree_;
    bool bGotGridmap_;
};
#endif //RRT_TRAJECTORYCOMBINER_H
