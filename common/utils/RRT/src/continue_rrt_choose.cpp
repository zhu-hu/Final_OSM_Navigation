//
// Created by huyao on 18-8-24.
//

#include "rrt/crrt_choose.h"

namespace huyao_RRT{

    CRRTChoose::CRRTChoose(ros::NodeHandle* nh, tf::TransformListener *listener)
            : nh_(nh), bPublish_(false), listener_(listener) {

        // Init log here
        google::InitGoogleLogging("CRRTChoose");
        FLAGS_stderrthreshold = google::INFO;
        FLAGS_alsologtostderr = true;
        FLAGS_colorlogtostderr = true;
        FLAGS_log_dir = "/home/cyber/testLog/";

        AINFO << "Log initiate success! ";

        path1Sub_ = nh_->subscribe("/path_continue_rrt1", 1, &CRRTChoose::path1Callback, this);
        path2Sub_ = nh_->subscribe("/path_continue_rrt2", 1, &CRRTChoose::path2Callback, this);
        path3Sub_ = nh_->subscribe("/path_continue_rrt3", 1, &CRRTChoose::path3Callback, this);
        path4Sub_ = nh_->subscribe("/path_continue_rrt4", 1, &CRRTChoose::path4Callback, this);
        path5Sub_ = nh_->subscribe("/path_continue_rrt5", 1, &CRRTChoose::path5Callback, this);
        path6Sub_ = nh_->subscribe("/path_continue_rrt6", 1, &CRRTChoose::path6Callback, this);
        subStart_ = nh_->subscribe("/rrt_plan_start", 1, &CRRTChoose::StartCallback, this);

        trajectory_pub = nh_->advertise<cyber_msgs::LocalTrajList>("/best_continue_rrt_trajectory", 1);
        local_path_pub = nh->advertise<nav_msgs::Path>("/local_path_crrt",2);

        publishTime_ = ros::Time::now() + ros::Duration(0.02);

        AINFO << "Node CRRTChoose init done!";

        run();
    }

    CRRTChoose::~CRRTChoose() {
        AINFO << "Node CRRTChoose shut down!";
        google::ShutdownGoogleLogging();
    }


    void CRRTChoose::StartCallback(const geometry_msgs::PoseStampedPtr &start) {

        publishTime_ = ros::Time::now() + ros::Duration(0.02);
        pathMap_.clear();
        bPublish_ = true;

    }


    bool CRRTChoose::run() {
        ros::Rate r(100);
        ros::Time now;
        while(ros::ok()){
            ros::spinOnce();
            now = ros::Time::now();
//            if(now >= publishTime_)
//                std::cout << now << ";" << publishTime_ << std::endl;
            if( now >= publishTime_ && !pathMap_.empty() && bPublish_) {
                publishBestPath();
            }
            r.sleep();
        }
    }

    void CRRTChoose::path1Callback(const nav_msgs::PathConstPtr &path_in){
        if(path_in->poses.empty())
            return;
        pathMap_.insert(std::make_pair(path_in->poses.back().pose.position.z, path_in));
    }

    void CRRTChoose::path2Callback(const nav_msgs::PathConstPtr &path_in){
        if(path_in->poses.empty())
            return;
        pathMap_.insert(std::make_pair(path_in->poses.back().pose.position.z, path_in));
    }

    void CRRTChoose::path3Callback(const nav_msgs::PathConstPtr &path_in){
        if(path_in->poses.empty())
            return;
        pathMap_.insert(std::make_pair(path_in->poses.back().pose.position.z, path_in));
    }

    void CRRTChoose::path4Callback(const nav_msgs::PathConstPtr &path_in){
        if(path_in->poses.empty())
            return;
        pathMap_.insert(std::make_pair(path_in->poses.back().pose.position.z, path_in));
    }

    void CRRTChoose::path5Callback(const nav_msgs::PathConstPtr &path_in){
        if(path_in->poses.empty())
            return;
        pathMap_.insert(std::make_pair(path_in->poses.back().pose.position.z, path_in));
    }

    void CRRTChoose::path6Callback(const nav_msgs::PathConstPtr &path_in){
        if(path_in->poses.empty())
            return;
        pathMap_.insert(std::make_pair(path_in->poses.back().pose.position.z, path_in));
    }


    void CRRTChoose::publishBestPath() {

        nav_msgs::Path::ConstPtr best_path = pathMap_.begin()->second;
        nav_msgs::Path path_to_pub = *best_path;
        path_to_pub.poses.pop_back();

        cyber_msgs::LocalTrajList pub_tj_list;
        cyber_msgs::LocalTrajPoint tj_point;
        geometry_msgs::PoseStamped world_pose;
        for(auto pose : path_to_pub.poses) {
            if (!TransformPose(listener_, "base_link", "world", pose, world_pose)) {
                return;///TF转换失败
            };
            tj_point.position = world_pose.pose.position;
            tj_point.theta = tf::getYaw(world_pose.pose.orientation);
            pub_tj_list.points.emplace_back(tj_point);
        }

        trajectory_pub.publish(pub_tj_list);
        local_path_pub.publish(path_to_pub);

        bPublish_ = false;
    }



}
