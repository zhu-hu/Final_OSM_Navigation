// Created by localization on 11/4/19.
//实现地图更新，定位状态更新

#ifndef SRC_GRID_MAP_LOCALIZATION_H
#define SRC_GRID_MAP_LOCALIZATION_H


#include <cstdio>
#include <iomanip>
#include <thread>
#include <mutex>
#include <condition_variable>

// For ROS output.
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Polygon.h>
#include <mrpt_bridge/mrpt_bridge.h>
#include <pcl_conversions/pcl_conversions.h>


#include "map_matching/mrpt_icp_matching.h"
#include "map_matching/pcl2mrpt.h"
#include "map_management/grid_map_2d.h"
#include "dead_reckoning/dead_reckoning_2d.h"


#include "common/struct/PoseStamped.h"
#include "common/struct/Pose2DStamped.h"
#include "common/struct/TwistStamped.h"
#include "common/struct/Pose3D.h"
#include "common/struct/BoundingBox.h"
#include "common/struct/StateEstimation.h"
#include "common/util/pointcloud_util.h"
#include "common/util/time_util.h"


namespace localization {

    enum LocalizationStatus {
        WAITING,
        INIT,
        MATCHING
    };

    enum InitializationMode {
        AUTO = 0,
        MANUAL
    };

    class GridMapLocalization{
    public:
        GridMapLocalization();
        //初始化
        ~GridMapLocalization(){
            delete matching_result_;
            delete matching_init_;
            delete cloud_pose_result_;
        }
        inline void SetLidarCaliInfo(Pose3D info) {
            lidar_calibration_info_ = info;
        }

        inline void SetCloudBoundingBox(BoundingBox box) {
            cloud_bounding_box_ = box;
        }

        inline void SetVoxelgridFilterSize(double size) {
            voxelgrid_filter_size_ = size;
        }

        inline void SetGridMapInfo(int grid_map_size,
                                   double grid_map_resolution,
                                   std::string grid_map_folder_path) {
            grid_map_.InitGridMap(grid_map_size,
                                  grid_map_resolution,
                                  grid_map_folder_path);
        }

        inline void UpdatePointCloud(util::PointCloudTypePtr &cloud, double time){
            point_cloud_ = cloud;
            if(cloud_pose_result_==nullptr)
                cloud_pose_result_ = new Pose2DStamped;
            cloud_pose_result_->timestamp=time;
        }
        inline void UpdatePointCloudPose(localization::Pose2D pose){
            if(cloud_pose_result_==nullptr)
                cloud_pose_result_ = new Pose2DStamped;
            cloud_pose_result_->pose.x=pose.x;
            cloud_pose_result_->pose.y=pose.y;
            cloud_pose_result_->pose.phi=pose.phi;
            IsPointCloudUpdate = true;
        }
    
        inline const Pose2DStamped *GetPointCloudMatchingResult() {
            return cloud_pose_result_;
        }

        inline void  SetMatchingInitPose(const double &x, const double &y, double &angle) {
            if(matching_init_==nullptr)
                matching_init_ = new Pose2DStamped;
            matching_init_->pose.x=x;
            matching_init_->pose.y=y;
            matching_init_->pose.phi=angle;
        }
        inline void  SetMatchingInitPose(const double &x, const double &y) {
            if(matching_init_==nullptr)
                matching_init_ = new Pose2DStamped;
            matching_init_->pose.x=x;
            matching_init_->pose.y=y;
        }
        inline void  SetMatchingInitPose(const localization::StateEstimation poseture) {
            if(matching_init_==nullptr)
                matching_init_ = new Pose2DStamped;
            double temp_rx,temp_ry,temp_rz;
            tf::Matrix3x3(tf::Quaternion(poseture.pose.orientation.x,
                                         poseture.pose.orientation.y,
                                         poseture.pose.orientation.z,
                                         poseture.pose.orientation.w))
                        .getRPY(temp_rx,temp_ry,temp_rz);
            matching_init_->pose.x=poseture.pose.position.x;
            matching_init_->pose.y=poseture.pose.position.y;
            matching_init_->pose.phi=temp_rz;
        }

        inline sensor_msgs::PointCloud2 GetPointCloud(const std::string &frame_id) {

            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*point_cloud_filtered_, cloud_msg);
            cloud_msg.header.frame_id = frame_id;
            return cloud_msg;
        }
        inline nav_msgs::OccupancyGrid GetRosGridMap(const std::string &frame_id) {
            nav_msgs::OccupancyGrid map_msg;
            if (matcher_.IsMapReady())
                mrpt_bridge::convert(matcher_.GetMap(), map_msg);
            map_msg.header.frame_id = frame_id;
            return map_msg;
        }
        inline void LoadGridMap(double x,double y){
            const int new_x = grid_map_.GetSquareCenterCoordinate(x);
            const int new_y = grid_map_.GetSquareCenterCoordinate(y);
            grid_map_.LoadGridMap(new_x,new_y);
            matcher_.SetMap(grid_map_.map); 
        }

        bool IsGpsPoseNan(Pose gps_pose);
        
        bool PointCloudMatch();
        
        void UpdateMap(double center_x,double center_y);

        bool LoadSlamArea(std::string file);

        bool IsInCL(Pose gps_pose);

        bool IsInCLS(Pose gps_pose);

        bool IsInSlamArea(Pose gps_pose);

        void PreprocessPointCloud();

        bool IsPointCloudUpdate;
        
        typedef struct
        {
            double x;
            double y;
        }stu_Slam_Corner;
        std::vector<std::vector<stu_Slam_Corner>> vec_Slam_Area_cp_;
        std::vector<std::vector<stu_Slam_Corner>> vec_Slam_Area;
        std::vector<stu_Slam_Corner> vec_CL_Area,vec_CLS_Area;

    private:
        Pose3D lidar_calibration_info_;
        BoundingBox cloud_bounding_box_;
        double voxelgrid_filter_size_;

        Pose2DStamped *matching_result_;
        Pose2DStamped *matching_init_;
        Pose2DStamped *cloud_pose_result_;
        Pose2DStamped *cloud_pose_;

        GridMap2D grid_map_;

        util::PointCloudTypePtr point_cloud_;
        double point_cloud_time_;
        util::PointCloudTypePtr point_cloud_filtered_;
        double point_cloud_filtered_time_;

        GridMapMatching matcher_;

        std::mutex matching_result_mutex_;

        double map_center_x_,map_center_y_;

        void Matching();

        //void PreprocessPointCloud(util::PointCloudTypePtr &cloud);
        
    };
}
#endif //SRC_GRID_MAP_LOCALIZATION_H