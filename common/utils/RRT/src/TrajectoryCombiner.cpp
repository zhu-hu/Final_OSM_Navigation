//
// Created by cyber on 18-10-19.
//

#include "TrajectoryCombiner.h"


TrajectoryCombine::TrajectoryCombine(ros::NodeHandle* nh, tf::TransformListener* listener)
            :nh_(nh), maxSpeed_(1), listener_(listener){
    // Init log here
    google::InitGoogleLogging("combine");
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_dir = "/home/cyber/testLog/";
    AINFO << "Log initiate success! ";

    subCompleteRRT_ = nh_->subscribe("/best_rrt_trajectory", 1, &TrajectoryCombine::CompleteRRTCallback, this);
    subGridMap_ = nh_->subscribe("/dilated_grid_map", 1, &TrajectoryCombine::GridMapCallback, this);
//    subIncompleteRRT_ = nh_->subscribe("/best_continue_rrt_trajectory", 1, &TrajectoryCombine::InCompleteRRTCallback, this);
    subCurrentPose_ = nh_->subscribe("/localization/estimation", 1, &TrajectoryCombine::CurrentPoseCallback, this);
//    pubStartSecTraj_ = nh_->advertise<geometry_msgs::PoseStamped>("/rrt_plan_start",1);
    subTaskGoal_ = nh_->subscribe("/DSMap/task_goal", 1, &TrajectoryCombine::TaskGoalCallback, this);
    pubRRT_ = nh_->advertise<cyber_msgs::LocalTrajList>("/rrt_trajectory", 1);
    pubRRTRviz_ = nh_->advertise<nav_msgs::Path>("/rrt_trajectory_rviz", 1);
}

TrajectoryCombine::~TrajectoryCombine() {}

void TrajectoryCombine::GridMapCallback(const sensor_msgs::ImageConstPtr &map_in) {
    cv_bridge::toCvShare(map_in, "mono8")->image.copyTo(localMap_);
    bGotGridmap_ = true;
}

void TrajectoryCombine::TaskGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal) {
    currentGoal_ = *goal;
    AWARN << "goal: " << currentGoal_.pose.position.x << "," << currentGoal_.pose.position.y;
}

void TrajectoryCombine::CurrentPoseCallback(const cyber_msgs::LocalizationEstimate &cp) {
    currentPose_.pose = cp.pose;
}

double CalculateTrajectoryLength(cyber_msgs::LocalTrajList &traj_in) {
    if(traj_in.points.size() <= 2) {
        return 0.0;
    }
    auto iter = traj_in.points.begin();
    double rest_s = 0.0;
    while(iter < traj_in.points.end() - 1)
    {
        double dis =  sqrt(pow((iter+1)->position.x - iter->position.x , 2) +
                                   pow((iter+1)->position.y - iter->position.y , 2));

        iter->s = rest_s;
        rest_s += dis;
        iter++;
    }
    traj_in.points.back().s = rest_s;
    AINFO << "rest_s" << rest_s;
    return rest_s;
}

void TrajectoryCombine::CompleteRRTCallback(const cyber_msgs::LocalTrajList::ConstPtr &traj_in) {

    newTrajectory_ = *traj_in;

//    AINFO << "trajectory_" << trajectory_.points.size() << ";" << "newTrajectory_" << newTrajectory_.points.size();

    if(trajectory_.points.empty() || !CutTrajectory(trajectory_) || CalculateTrajectoryLength(trajectory_)< 25.0) {
        AWARN << "dist 2 goal: " << huyao_rrt::dist2Points(currentGoal_, currentPose_);
        if((CalculateTrajectoryLength(newTrajectory_) > 10.0)||(huyao_rrt::dist2Points(currentGoal_, currentPose_) < 12.0)) {
            trajectory_ = newTrajectory_;
            std::cout << "use new path" << std::endl;
        }
        else{
            trajectory_.points.clear();
        }
    }

//    AINFO << "trajectory_" << trajectory_.points.size() ;

    if(!CheckTrajFree(trajectory_)) {
        AERROR << "Final Trajectory is not free! ";
        trajectory_.points.clear();
    }

    pubRRT_.publish(trajectory_);
    ///将路径显示到rviz
    nav_msgs::Path path;
    geometry_msgs::PoseStamped cell;
    path.header.frame_id = "world";
    path.header.stamp = ros::Time::now();
    for(auto pt : trajectory_.points){
        cell.pose.position.x = pt.position.x;
        cell.pose.position.y = pt.position.y;
        cell.pose.position.z = 0;
        path.poses.emplace_back(cell);
    }
    pubRRTRviz_.publish(path);
}

//void TrajectoryCombine::CompleteRRTCallback(const cyber_msgs::LocalTrajList::ConstPtr &traj_in) {
//    if (CheckTrajGrad(maxSpeed_)) {
//        ROS_WARN("Current path is OK!");
//    } else {
//        maxSpeed_ = 0.0;
//        bSecTrajIsValid_ = false;
//        trajectory_ = *traj_in;
//        //EstimateKappa();
//        newTrajectory_ = trajectory_;
//        bNeedCut_ = true;
//    }
//    pubRRT_.publish(newTrajectory_);
//    ///将路径显示到rviz
//    nav_msgs::Path path;
//    geometry_msgs::PoseStamped cell;
//    path.header.frame_id = "world";
//    path.header.stamp = ros::Time::now();
//    for(auto pt : newTrajectory_.points){
//        cell.pose.position.x = pt.position.x;
//        cell.pose.position.y = pt.position.y;
//        cell.pose.position.z = 0;
//        path.poses.emplace_back(cell);
//    }
//    pubRRTRviz_.publish(path);
//}

void TrajectoryCombine::InCompleteRRTCallback(const cyber_msgs::LocalTrajList::ConstPtr &traj_in) {
    if(!bFirstTrajIsFree_)
        return ;

    if(traj_in->points.empty()){
        AWARN << "[" << ros::Time::now()<< "]" << "plan failed";
        return ;
    }

    secTrajectory_ = *traj_in;
    //EstimateKappa(secTrajectory_);
    bSecTrajIsValid_ = true;
}

bool TrajectoryCombine::CheckTrajGrad(float &speed) {
    if(trajectory_.points.empty()) {return false;}
    if(!bGotGridmap_) {return false;}
    /// find nearest point
    auto iter = trajectory_.points.begin();
    double temp_dis = std::numeric_limits<double>::infinity();
    while (iter < trajectory_.points.end()) {
        double next_dis = pow(iter->position.y - currentPose_.pose.position.y, 2) +
                          pow(iter->position.x - currentPose_.pose.position.x, 2);
        if (next_dis <= temp_dis) {
            temp_dis = next_dis;
        } else {
            break;
        }
        iter++;
    }
    auto nearestIter = iter;
    if (temp_dis > 1.5*1.5) {
        AERROR << "[" << ros::Time::now()<< "]"  << "Offset too large!";
        AERROR << "Offset is: " << sqrt(temp_dis);
        return false;
    }

    if(bNeedCut_) {
        firstTrajectory_ = trajectory_;
        while (nearestIter < trajectory_.points.end()) {
            double next_dis = pow(nearestIter->position.y - currentPose_.pose.position.y, 2) +
                              pow(nearestIter->position.x - currentPose_.pose.position.x, 2);
            if (next_dis > kCutDistance * kCutDistance ) {
                break;
            }
            nearestIter++;
        }

        if(nearestIter < trajectory_.points.end()){
            trajectory_.points.erase(nearestIter,trajectory_.points.end());
            std::swap(firstTrajectory_, trajectory_);
        }
        PubCRRTStart();
        AWARN << "[" << ros::Time::now()<< "]"  << "update first segement" ;
        bNeedCut_ = false;
        AINFO<<firstTrajectory_.points.size();
    } else {
        if(bSecTrajIsValid_ && (pow(firstTrajectory_.points.back().position.x - secTrajectory_.points.front().position.x , 2)
                                + pow(firstTrajectory_.points.back().position.y - secTrajectory_.points.front().position.y , 2) < 0.5 )) {


            newTrajectory_ = firstTrajectory_;
            newTrajectory_.points.insert(newTrajectory_.points.end(), secTrajectory_.points.begin(),
                                          secTrajectory_.points.end());
            AWARN << "[" << ros::Time::now()<< "]"   << "segement is linked" ;
        } else {
            ROS_WARN("5#");
            newTrajectory_ = trajectory_;
            bNeedCut_ = true;
            speed = kMaxSpeed;
        }
    }
    if(!CheckTrajFree(firstTrajectory_)){
        AWARN << "[" << ros::Time::now()<< "]"   << "first segement is not free" ;
        return false;
    }
    bFirstTrajIsFree_ = true;
    if(!CheckTrajFree(newTrajectory_)){
        newTrajectory_ = trajectory_;
        bSecTrajIsValid_ = false;
        bNeedCut_ = true;
        speed = kMaxSpeed;
        AWARN << "[" << ros::Time::now()<< "]"   << "second segement is not free" ;
    }

    double disToTrajEnd = pow(newTrajectory_.points.back().position.x - currentPose_.pose.position.x,2)
                          + pow(newTrajectory_.points.back().position.y - currentPose_.pose.position.y,2);
    if(disToTrajEnd < kRemainDis * kRemainDis) {
        AWARN <<"[" << ros::Time::now()<< "]"  << "remain dis too short" ;
        return false;
    }

    if(bSecTrajIsValid_){
        double disToSecond = pow(secTrajectory_.points.front().position.x - currentPose_.pose.position.x,2)
                             + pow(secTrajectory_.points.front().position.y - currentPose_.pose.position.y,2);

        double disToFirst = pow(firstTrajectory_.points.back().position.x - currentPose_.pose.position.x,2)
                            + pow(firstTrajectory_.points.back().position.y - currentPose_.pose.position.y,2);
        AWARN << "[" << ros::Time::now()<< "]"   << " " <<  disToSecond << "," << disToFirst;
        if((disToSecond <= 1.5*1.5) &&(disToFirst >= disToSecond)) {
            trajectory_ = secTrajectory_;
            bSecTrajIsValid_ = false;
            bNeedCut_ = true;
            AWARN << "[" << ros::Time::now()<< "]"   << " change" ;
        }
    }
    speed = kMaxSpeed;
    return true;
}

void TrajectoryCombine::PubCRRTStart(){

    geometry_msgs::PoseStamped rrt_start;
    rrt_start.header.frame_id = "world";
    rrt_start.pose.position.x = firstTrajectory_.points.back().position.x;
    rrt_start.pose.position.y = firstTrajectory_.points.back().position.y;
    rrt_start.pose.orientation = tf::createQuaternionMsgFromYaw(firstTrajectory_.points.back().theta);

    AWARN << "[" << ros::Time::now()<< "]"   << "CRRT start in ( " <<  firstTrajectory_.points.back().position.x << " , "
          << firstTrajectory_.points.back().position.y << " , " << firstTrajectory_.points.back().theta << " )"  ;

    pubStartSecTraj_.publish(rrt_start);
}

bool TrajectoryCombine::CutTrajectory(cyber_msgs::LocalTrajList& traj_in) {

    if(traj_in.points.empty())
        return false;

    auto iter = traj_in.points.begin();
    auto nearestIter = iter;
    double temp_dis = std::numeric_limits<double>::infinity();
    while (iter < traj_in.points.end()) {
        double next_dis = pow(iter->position.y - currentPose_.pose.position.y, 2) +
                          pow(iter->position.x - currentPose_.pose.position.x, 2);
        if (next_dis <= temp_dis) {
            temp_dis = next_dis;
            nearestIter = iter;
        } else {
            break;
        }
        iter++;
    }

//    auto nearestIter = iter;
    AERROR << "Offset is: " << sqrt(temp_dis);

    if (temp_dis > 1.5*1.5) {
        AERROR << "[" << ros::Time::now()<< "]"  << "Offset too large!";
//        AERROR << "Offset is: " << sqrt(temp_dis);
        traj_in.points.clear();
        return false;
    }
    traj_in.points.erase(traj_in.points.begin(), nearestIter);

    return true;
}


bool TrajectoryCombine::CheckTrajFree(const cyber_msgs::LocalTrajList& traj_in) {
    geometry_msgs::PointStamped point_w, point_b;
    double pix_x, pix_y;

    if(traj_in.points.size() < 2) {
        return false;
    }

    auto iter = traj_in.points.begin();
    while (iter < traj_in.points.end()){
        point_w.point.x = iter->position.x;
        point_w.point.y = iter->position.y;
        if(TransformPoint(listener_, "world", "base_link", point_w, point_b)) {
            TFXY2PixInCar(point_b.point.x, point_b.point.y, pix_x, pix_y);
            if (0 <= pix_x && pix_x < map_param::grid_map::kWidth && 0 <= pix_y &&
                pix_y < map_param::grid_map::kHeight) {
                if (localMap_.at<uchar>(pix_y, pix_x) > map_param::grid_map::kObstacleThreshold) {
                    return false;
                }
            }
            iter++;
        } else {
            return false;
        }
    }
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rrt");
    ros::NodeHandle pnh("~");
    tf::TransformListener *listener = new tf::TransformListener();

    TrajectoryCombine rrt(&pnh, listener);

    ros::spin();
    return 0;
}