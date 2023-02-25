//
// Created by huyao on 18-8-24.
//

#include <std_msgs/Float32.h>
#include "rrt/rrt_choose.h"

namespace huyao_RRT{

    RRTChoose::RRTChoose(ros::NodeHandle* nh, tf::TransformListener *listener)
            : nh_(nh), bPublish_(false), listener_(listener), bReverse_(false),
                bGotGoal_(false), timer_count_(0) {

        // Init log here
        google::InitGoogleLogging("RRTChoose");
        FLAGS_stderrthreshold = google::INFO;
        FLAGS_alsologtostderr = true;
        FLAGS_colorlogtostderr = true;
        FLAGS_log_dir = "/home/cyber/testLog/";

        AINFO << "Log initiate success! ";

        time_ = nh->createTimer(ros::Duration(1.0), &RRTChoose::TimerCallback, this);

        path1Sub_ = nh_->subscribe("/path_rrt1", 1, &RRTChoose::path1Callback, this);
        path2Sub_ = nh_->subscribe("/path_rrt2", 1, &RRTChoose::path2Callback, this);
        path3Sub_ = nh_->subscribe("/path_rrt3", 1, &RRTChoose::path3Callback, this);
        path4Sub_ = nh_->subscribe("/path_rrt4", 1, &RRTChoose::path4Callback, this);
        path5Sub_ = nh_->subscribe("/path_rrt5", 1, &RRTChoose::path5Callback, this);
        path6Sub_ = nh_->subscribe("/path_rrt6", 1, &RRTChoose::path6Callback, this);

        goalSub_ = nh_->subscribe("/DStar/local_target", 1, &RRTChoose::goalCallback, this);
//        goalSub_ = nh_->subscribe("/move_base_simple/goal", 1, &RRTChoose::goalCallback, this);


        iT_ = new image_transport::ImageTransport(*nh_);
        subGridMap_ = iT_->subscribe("/dilated_grid_map", 1, &RRTChoose::GridMapCallback, this);
        trajectory_pub = nh_->advertise<cyber_msgs::LocalTrajList>("/best_rrt_trajectory", 1);
        local_path_pub = nh->advertise<nav_msgs::Path>("/local_path",2);
        rrt_goal_pub = nh->advertise<geometry_msgs::PoseStamped>("/rrt_choose_local_target",2);


        publishTime_ = ros::Time::now() + ros::Duration(0.02);

        AINFO << "Node RRTChoose init done!";

        run();
    }

    RRTChoose::~RRTChoose() {
        delete iT_;

        AINFO << "Node RRTChoose shut down!";
        google::ShutdownGoogleLogging();
    }

    void RRTChoose::goalCallback(const geometry_msgs::PoseStampedPtr &goal){

        goal_world_ = *goal;
        bGotGoal_ = true;
        timer_count_ = 0;
    }

    void RRTChoose::TimerCallback(const ros::TimerEvent& event){
        timer_count_ = std::min(20, timer_count_+1);
        bGotGoal_ = timer_count_ > 10 ? false : bGotGoal_;
    }


    void RRTChoose::GridMapCallback(const sensor_msgs::ImageConstPtr &map_in){
        cv_bridge::toCvShare(map_in, "mono8")->image.copyTo(localMap_);
//        cv::Mat kernelDilateL = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(map_param::grid_map::kDDilateSize, map_param::grid_map::kDDilateSize));
//        dilate(localMap_, localMap_, kernelDilateL);
//        threshold(localMap_, localMap_, map_param::grid_map::kObstacleThreshold, 255, CV_THRESH_BINARY);
//        cv::imshow("threshold",localMap_);
//        cv::waitKey(20);

        if(!bGotGoal_) {
            return;
        }

        geometry_msgs::PoseStamped local_map_pose;
        geometry_msgs::PoseStamped goal_w;
        goal_w.pose = goal_world_.pose;
//        ROS_ERROR("2#");
        if (!TransformPose(listener_, "world", "base_link", goal_w, local_map_pose)) {
            return;///TF转换失败
        };
//        ROS_ERROR("2#E");

//        DecideCarDirection(local_map_pose);
        rrt_goal_pub.publish(local_map_pose);

//        if(!findTargetMapPoint(local_map_pose)){
//
//        }

        // my update policy, to avoid delay
        publishTime_ = ros::Time::now() + ros::Duration(0.02);
        pathMap_.clear();
        bPublish_ = true;
    }

    bool RRTChoose::run() {
        ros::Rate r(100);
        ros::Time now;
        while(ros::ok()){
            ros::spinOnce();
            now = ros::Time::now();
            if( now >= publishTime_ && !pathMap_.empty() && bPublish_) {
                publishBestPath();
            }
            r.sleep();
        }
    }

    void RRTChoose::path1Callback(const nav_msgs::PathConstPtr &path_in){
        if(path_in->poses.empty())
            return;
        pathMap_.insert(std::make_pair(path_in->poses.back().pose.position.z, path_in));
    }

    void RRTChoose::path2Callback(const nav_msgs::PathConstPtr &path_in){
        if(path_in->poses.empty())
            return;
        pathMap_.insert(std::make_pair(path_in->poses.back().pose.position.z, path_in));
    }

    void RRTChoose::path3Callback(const nav_msgs::PathConstPtr &path_in){
        if(path_in->poses.empty())
            return;
        pathMap_.insert(std::make_pair(path_in->poses.back().pose.position.z, path_in));
    }

    void RRTChoose::path4Callback(const nav_msgs::PathConstPtr &path_in){
        if(path_in->poses.empty())
            return;
        pathMap_.insert(std::make_pair(path_in->poses.back().pose.position.z, path_in));
    }

    void RRTChoose::path5Callback(const nav_msgs::PathConstPtr &path_in){
        if(path_in->poses.empty())
            return;
        pathMap_.insert(std::make_pair(path_in->poses.back().pose.position.z, path_in));
    }

    void RRTChoose::path6Callback(const nav_msgs::PathConstPtr &path_in){
        if(path_in->poses.empty())
            return;
        pathMap_.insert(std::make_pair(path_in->poses.back().pose.position.z, path_in));
    }

    float getCloset(std::vector<float> index, float ref){

        if(index.empty()) return std::numeric_limits<float>::max();
        auto it = lower_bound(index.begin(), index.end(), ref);
        if(it == index.begin()) return *it;
        else if(it == index.end()) return std::numeric_limits<float>::max();
        if( (*it - ref) > (ref - *(it - 1)) ){
            return  *(it -1 );
        } else {
            return  *it;
        }
    }


//    bool isOnMap(const cyber_common::math::Vec3d & PointNeedTest) const{
//
//        std::cout << PointNeedTest.x << "," << PointNeedTest.y << std::endl;
//        std::cout << grid->info.width << "," << grid->info.height << std::endl;
//
//        if(PointNeedTest.y < 0.0 || PointNeedTest.y > grid->info.height - 1){
//            return false;
//        }
//        if(PointNeedTest.x < 0.0 || PointNeedTest.x > grid->info.width - 1){
//            return false;
//        }
//        return true;
//    }


    bool RRTChoose::findTargetMapPointReverse(const geometry_msgs::PoseStamped &point_in) {


        double target_theta = tf::getYaw(point_in.pose.orientation);

        if(bReverse_) {
            if(std::fabs(target_theta) < task::kReverseBackToNormal){
                bReverse_ = false;
            }
        } else {
            if(std::fabs(target_theta) > task::kReverseThreshold){
                bReverse_ = true;
            }
        }

        std_msgs::Float32 dir;
        if(bReverse_){
            dir.data = -1.0;
        } else {
            dir.data = 1.0;
        }

        int pix_x,pix_y;

        std::cout << "(" << point_in.pose.position.x << ";" << point_in.pose.position.y << ")" << std::endl;

        TFXY2PixInCar(point_in.pose.position.x, point_in.pose.position.y,pix_x,pix_y);

        std::cout << "(" << pix_x << ";" << pix_y << ")" << std::endl;

        int kernel_size = 2*map_param::grid_map::kCarRadius;
        cyber_common::math::Vec2d   vector_start(map_param::grid_map::kCarCenterX, map_param::grid_map::kCarCenterY),
                vector_left_down(0.0, 0.0),
                vector_left_top(0.0, localMap_.rows - 1.5),
                vector_right_top(localMap_.cols - 1.5, localMap_.rows - 1.5),
                vector_right_down(localMap_.cols - 1.5, 0.0),
                vector_end(pix_x, localMap_.rows - 1 - pix_y),
                initPoint;


        int pix_out_x, pix_out_y;
        double theta;

        // if target on map
        if( 0 <= pix_x && pix_x <= localMap_.cols - 1 && 0 <= pix_y && pix_y <= localMap_.rows - 1 ){

            pix_out_x = pix_x;
            pix_out_y = pix_y;
            theta = tf::getYaw(point_in.pose.orientation);

        } else {

            if (twoVectorCross(vector_start, vector_end, vector_left_down, vector_left_top)) {
                initPoint = crossPoint(vector_start, vector_end, vector_left_down, vector_left_top);
            } else if (twoVectorCross(vector_start, vector_end, vector_left_top, vector_right_top)) {
                initPoint = crossPoint(vector_start, vector_end, vector_left_top, vector_right_top);
            } else if (twoVectorCross(vector_start, vector_end, vector_right_top, vector_right_down)) {
                initPoint = crossPoint(vector_start, vector_end, vector_right_top, vector_right_down);
            }

            pix_out_x = initPoint.x();
            pix_out_y = localMap_.rows - 1 - initPoint.y();
            theta = atan2(initPoint.y() - map_param::grid_map::kCarCenterY,
                          pix_out_x - map_param::grid_map::kCarCenterX);
            theta = cyber_common::math::NormalizeAngle(theta - M_PI/2);
        }

        geometry_msgs::PoseStamped local_target;
        double local_x, local_y;
        TFPix2XYInCar(pix_out_x, pix_out_y, local_x, local_y);
        local_target.header.frame_id = "base_link";
        local_target.pose.position.x = local_x;
        local_target.pose.position.y = local_y;
        tf::Quaternion quater = tf::createQuaternionFromYaw(theta);
        local_target.pose.orientation.x = quater.x();
        local_target.pose.orientation.y = quater.y();
        local_target.pose.orientation.z = quater.z();
        local_target.pose.orientation.w = quater.w();

        rrt_goal_pub.publish(local_target);

        return true;
    }

    void RRTChoose::DecideCarDirection(const geometry_msgs::PoseStamped &point_in) {

        double target_theta = tf::getYaw(point_in.pose.orientation);

        if(bReverse_) {
            ROS_ERROR("target_theta: %f and threshold: %f", target_theta, task::kReverseBackToNormal);
            if(std::fabs(target_theta) < task::kReverseBackToNormal){
                bReverse_ = false;
            }
        } else {
            if(std::fabs(target_theta) > task::kReverseThreshold){
                bReverse_ = true;
            }
        }

        std_msgs::Float32 dir;
        if(bReverse_){
            dir.data = -1.0;
        } else {
            dir.data = 1.0;
        }
    }

    bool RRTChoose::findTargetMapPoint(const geometry_msgs::PoseStamped &point_in) {

        double target_theta = tf::getYaw(point_in.pose.orientation);

        if(bReverse_) {
            if(std::fabs(target_theta) < task::kReverseBackToNormal){
                bReverse_ = false;
            }
        } else {
            if(std::fabs(target_theta) > task::kReverseThreshold){
                bReverse_ = true;
            }
        }

        std_msgs::Float32 dir;
        if(bReverse_){
            dir.data = -1.0;
        } else {
            dir.data = 1.0;
        }
//        if(bReverse_) {
//            return false;
//        }

        int pix_x,pix_y;

        std::cout << "(" << point_in.pose.position.x << ";" << point_in.pose.position.y << ")" << std::endl;

        TFXY2PixInCar(point_in.pose.position.x, point_in.pose.position.y,pix_x,pix_y);

        std::cout << "(" << pix_x << ";" << pix_y << ")" << std::endl;

        int kernel_size = 2*map_param::grid_map::kCarRadius;
        cyber_common::math::Vec2d   vector_start(map_param::grid_map::kCarCenterX, map_param::grid_map::kCarCenterY),
                vector_left_down(0.0, 0.0),
                vector_left_top(0.0, localMap_.rows - 1.5),
                vector_right_top(localMap_.cols - 1.5, localMap_.rows - 1.5),
                vector_right_down(localMap_.cols - 1.5, 0.0),
                vector_end(pix_x, localMap_.rows - 1 - pix_y),
                initPoint;

//        std::cout << "grid height : " << localMap_.rows << std::endl;
//        std::cout  << "grid width : " << localMap_.cols << std::endl;
//        std::cout  << "top:" << vector_left_top.x() << "," << vector_right_top.y() << std::endl;

//        std::vector<float> index_left;
//        std::vector<float> index_top;
//        std::vector<float> index_right;
//
//        cv::Mat kernel = cv::Mat::ones(kernel_size, kernel_size, CV_8U);
//        cv::Mat dst;
//        filter2D(localMap_, dst, -1 , kernel, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT );
//
//
//        for(int i = 0; i < localMap_.rows; i++){
//            if (localMap_.at<uchar>(i, 1) == 0){
//                index_left.emplace_back( i );
//            }
//            if (localMap_.at<uchar>(i, localMap_.cols-2) == 0){
//                index_right.emplace_back( i );
//            }
//        }
//        for (int i = 0; i < localMap_.cols; ++i) {
//            if (localMap_.at<uchar>(1,i) == 0){
//                index_top.emplace_back( i );
//            }
//        }
//
//
//        float index;
//        if(twoVectorCross(vector_start, vector_end, vector_left_down, vector_left_top)){
//            initPoint = crossPoint(vector_start, vector_end, vector_left_down, vector_left_top);
//            index = getCloset(index_left, float(initPoint.y()));
//            index = std::max( float(0.5), index - float(0.5));
//            initPoint.set_y(index);
//        } else if(twoVectorCross(vector_start, vector_end, vector_left_top, vector_right_top)){
//            initPoint = crossPoint(vector_start, vector_end, vector_left_top, vector_right_top);
//            index = getCloset(index_top,  float(initPoint.x()));
//            index = std::max( float(0.5), index - float(0.5));
//            initPoint.set_x(index);
//        } else if(twoVectorCross(vector_start, vector_end, vector_right_top, vector_right_down)){
//            initPoint = crossPoint(vector_start, vector_end, vector_right_top, vector_right_down);
//            index = getCloset(index_right,  float(initPoint.y()));
//            index = std::max( float(0.5), index - float(0.5));
//            initPoint.set_y(index);
//        }
//        if(index > localMap_.cols || index > localMap_.rows) {
//            return false;
//        }
//        else {
//            int pix_out_x = initPoint.x();
//            int pix_out_y = initPoint.y();
//            double theta = atan2(initPoint.y()-map_param::grid_map::kCarCenterY, initPoint.x() - map_param::grid_map::kCarCenterX);
//            theta = cyber_common::math::NormalizeAngle(theta);
//
//            geometry_msgs::PoseStamped local_target;
//            double local_x, local_y;
//            TFPix2XYInCar(pix_out_x, localMap_.rows -1 - pix_out_y, local_x, local_y);
//            local_target.header.frame_id = "base_link";
//            local_target.pose.position.x = local_x;
//            local_target.pose.position.y = local_y;
//            tf::Quaternion quater = tf::createQuaternionFromYaw(theta);
//            local_target.pose.orientation.x = quater.x();
//            local_target.pose.orientation.y = quater.y();
//            local_target.pose.orientation.z = quater.z();
//            local_target.pose.orientation.w = quater.w();
//
//            rrt_goal_pub.publish(local_target);
//
//            std::cout  << "Edge target point has been calculated, at ("
//                       << pix_out_x << "," << pix_out_y << "," << theta << ")." << std::endl;
//        }


        int pix_out_x, pix_out_y;
        double theta;

        // if target on map
        if( 0 <= pix_x && pix_x <= localMap_.cols - 1 && 0 <= pix_y && pix_y <= localMap_.rows - 1 ){

            pix_out_x = pix_x;
            pix_out_y = pix_y;
            theta = tf::getYaw(point_in.pose.orientation);

        } else {

            if (twoVectorCross(vector_start, vector_end, vector_left_down, vector_left_top)) {
                initPoint = crossPoint(vector_start, vector_end, vector_left_down, vector_left_top);
            } else if (twoVectorCross(vector_start, vector_end, vector_left_top, vector_right_top)) {
                initPoint = crossPoint(vector_start, vector_end, vector_left_top, vector_right_top);
            } else if (twoVectorCross(vector_start, vector_end, vector_right_top, vector_right_down)) {
                initPoint = crossPoint(vector_start, vector_end, vector_right_top, vector_right_down);
            }

            pix_out_x = initPoint.x();
            pix_out_y = localMap_.rows - 1 - initPoint.y();
            theta = atan2(initPoint.y() - map_param::grid_map::kCarCenterY,
                          pix_out_x - map_param::grid_map::kCarCenterX);
//            std::cout << "tangent:" << theta << std::endl;
            theta = cyber_common::math::NormalizeAngle(theta - M_PI/2);
        }

        geometry_msgs::PoseStamped local_target;
        double local_x, local_y;
        TFPix2XYInCar(pix_out_x, pix_out_y, local_x, local_y);
        local_target.header.frame_id = "base_link";
        local_target.pose.position.x = local_x;
        local_target.pose.position.y = local_y;
        tf::Quaternion quater = tf::createQuaternionFromYaw(theta);
        local_target.pose.orientation.x = quater.x();
        local_target.pose.orientation.y = quater.y();
        local_target.pose.orientation.z = quater.z();
        local_target.pose.orientation.w = quater.w();

        rrt_goal_pub.publish(local_target);

//        std::cout  << "Edge target point has been calculated, at ("
//              << pix_out_x << "," << pix_out_y << "," << theta << ")." << std::endl;

        return true;
    }



    void RRTChoose::publishBestPath() {

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
