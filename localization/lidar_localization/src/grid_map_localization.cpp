//
// Created by localization on 11/4/19.
//

#include "grid_map_localization.h"

namespace localization {

    GridMapLocalization::GridMapLocalization() {

        matching_result_ = new Pose2DStamped;
        matching_init_= new Pose2DStamped;
        cloud_pose_result_ = nullptr;

        point_cloud_.reset(new util::PointCloudType());
        point_cloud_filtered_.reset(new util::PointCloudType());

        IsPointCloudUpdate = false;

        vec_CL_Area.push_back({-20486,-7589});
        vec_CL_Area.push_back({-20498,-7580});
        vec_CL_Area.push_back({-20200,-7580});
        vec_CL_Area.push_back({-20200,-7590});
        vec_CL_Area.push_back({-20486,-7589});

        vec_CLS_Area.push_back({-20437,-7752});
        vec_CLS_Area.push_back({-20376,-7752});
        vec_CLS_Area.push_back({-20376,-7794});
        vec_CLS_Area.push_back({-20438,-7794});

        // 初始化 ICP参数

    }
    

    bool GridMapLocalization::LoadSlamArea(std::string file){
        FILE* fpSlam_Area = fopen(file.c_str(),"r");
        if (fpSlam_Area == NULL){
            std::cout<<"error!";
            return false;
        } 
        std::vector<stu_Slam_Corner> stu_Slam_Shape;
        float tempX,tempY;
        while (fscanf(fpSlam_Area,"%f,%f;",&tempX,&tempY)==2)
        {
            if(tempX!=-1 || tempY!=-1)
            {
                stu_Slam_Shape.push_back({tempX,tempY});
            }
            else
            {
                stu_Slam_Shape.push_back(stu_Slam_Shape[0]);
                vec_Slam_Area.push_back(stu_Slam_Shape);
                stu_Slam_Shape.clear();
            }
        }
        vec_Slam_Area_cp_=vec_Slam_Area;
        return true;
    }
    // void GridMapLocalization::LoadLargeScaleMap(int grid_map_size,
    //                            double grid_map_resolution,
    //                            std::string grid_map_folder_path,
    //                            float center_x,
    //                            float center_y){
    //     grid_map_.InitGridMap(grid_map_size,grid_map_resolution,grid_map_folder_path);
    //     grid_map_.UpdateGridMap(center_x, center_y);
    //     cloud_pose_ = new Pose2DStamped;
    //     cloud_pose_->pose.x = center_x;
    //     cloud_pose_->pose.y = center_y;
    //     cloud_pose_->pose.phi = 0;
    // }
    bool GridMapLocalization::IsGpsPoseNan(Pose gps_pose){

        if (std::isnan(gps_pose.position.x) ||
            std::isnan(gps_pose.position.y) ||
            std::isnan(gps_pose.position.z) ||
            std::isnan(gps_pose.orientation.x) ||
            std::isnan(gps_pose.orientation.y) ||
            std::isnan(gps_pose.orientation.z) ||
            std::isnan(gps_pose.orientation.w))
            return true;
        else
            return false;
    }
    void GridMapLocalization::UpdateMap(double center_x,double center_y) {
        //根据中心位置判断地图是否需要更新，更新mrpt匹配需要的地图。调用grid_map_2d函数
        map_center_x_ = center_x;
        map_center_y_ = center_y; 
        if (grid_map_.UpdateGridMap(map_center_x_,
                                    map_center_y_)) {
            matcher_.SetMap(grid_map_.map);  //matcher_中函数用于匹配
        }
    }
    
    bool GridMapLocalization::GridMapLocalization::IsInCL(Pose gps_pose){
        float pose_x = gps_pose.position.x;
        float pose_y = gps_pose.position.y;
        int nCount=0;
        for (int j=0; j<(int)vec_CL_Area.size()-1; j++)
        {   
            if (vec_CL_Area[j].x==pose_x && vec_CL_Area[j+1].x==pose_x)
            {
                return true;
            }
            else if (vec_CL_Area[j].x==vec_CL_Area[j+1].x)
            {
                continue;
            }
            else if ((vec_CL_Area[j].x > pose_x && vec_CL_Area[j+1].x < pose_x)||
                        (vec_CL_Area[j].x < pose_x && vec_CL_Area[j+1].x > pose_x))
            {
                float tempY = 
                    (pose_x-vec_CL_Area[j].x)
                    *
                    (vec_CL_Area[j+1].y-vec_CL_Area[j].y)
                    /
                    (vec_CL_Area[j+1].x-vec_CL_Area[j].x)
                    +
                    vec_CL_Area[j].y;
                if (abs(tempY-pose_y)<0.01)
                {
                    //printf("yuan%d\n",vec_CL_Area[j+1].x);
                    return true;
                }
                else if (tempY>pose_y)
                {
                        nCount++;
                }
            }
        }
        if (nCount%2==1)
        {
            //printf("%d\n",i);
            return true;
        }
        else
        {
            nCount=0;
        }
  
        //printf("0\n");
        return false;
    }

    bool GridMapLocalization::GridMapLocalization::IsInCLS(Pose gps_pose){
        float pose_x = gps_pose.position.x;
        float pose_y = gps_pose.position.y;
        int nCount=0;
        for (int j=0; j<(int)vec_CLS_Area.size()-1; j++)
        {   
            if (vec_CLS_Area[j].x==pose_x && vec_CLS_Area[j+1].x==pose_x)
            {
                return true;
            }
            else if (vec_CLS_Area[j].x==vec_CLS_Area[j+1].x)
            {
                continue;
            }
            else if ((vec_CLS_Area[j].x > pose_x && vec_CLS_Area[j+1].x < pose_x)||
                        (vec_CLS_Area[j].x < pose_x && vec_CLS_Area[j+1].x > pose_x))
            {
                float tempY = 
                    (pose_x-vec_CLS_Area[j].x)
                    *
                    (vec_CLS_Area[j+1].y-vec_CLS_Area[j].y)
                    /
                    (vec_CLS_Area[j+1].x-vec_CLS_Area[j].x)
                    +
                    vec_CLS_Area[j].y;
                if (abs(tempY-pose_y)<0.01)
                {
                    //printf("yuan%d\n",vec_CLS_Area[j+1].x);
                    return true;
                }
                else if (tempY>pose_y)
                {
                        nCount++;
                }
            }
        }
        if (nCount%2==1)
        {
            //printf("%d\n",i);
            return true;
        }
        else
        {
            nCount=0;
        }
  
        //printf("0\n");
        return false;
    }
    
    bool GridMapLocalization::IsInSlamArea(Pose gps_pose){
        float pose_x = gps_pose.position.x;
        float pose_y = gps_pose.position.y;
        int nCount=0;
        for (int i=0; i<vec_Slam_Area.size(); i++)
        {
            for (int j=0; j<(int)vec_Slam_Area[i].size()-1; j++)
            {   
                if (vec_Slam_Area[i][j].x==pose_x && vec_Slam_Area[i][j+1].x==pose_x)
                {
                    return true;
                }
                else if (vec_Slam_Area[i][j].x==vec_Slam_Area[i][j+1].x)
                {
                    continue;
                }
                else if ((vec_Slam_Area[i][j].x > pose_x && vec_Slam_Area[i][j+1].x < pose_x)||
                         (vec_Slam_Area[i][j].x < pose_x && vec_Slam_Area[i][j+1].x > pose_x))
                {
                    float tempY = 
                     (pose_x-vec_Slam_Area[i][j].x)
                    *
                    (vec_Slam_Area[i][j+1].y-vec_Slam_Area[i][j].y)
                    /
                    (vec_Slam_Area[i][j+1].x-vec_Slam_Area[i][j].x)
                    +
                    vec_Slam_Area[i][j].y;
                    if (abs(tempY-pose_y)<0.01)
                    {
                        //printf("yuan%d\n",vec_Slam_Area[i][j+1].x);
                        return true;
                    }
                    else if (tempY>pose_y)
                    {
                        nCount++;
                    }
                }
            }
            if (nCount%2==1)
            {
                //printf("%d\n",i);
                return true;
            }
            else
            {
                nCount=0;
            }
        }
        //printf("0\n");
        return false;
    }
    bool GridMapLocalization::PointCloudMatch(){
        matching_result_mutex_.lock();
        matching_result_->timestamp = cloud_pose_result_->timestamp;
        clock_t matching_starttime = clock();
        Matching();
        double matching_time = util::GetTimeInterval(matching_starttime);
        std::cout << "[lidar] Matching Time: " << matching_time * 1000 << " ms." << '\n';
        if (cloud_pose_result_ == nullptr)
            cloud_pose_result_ = new Pose2DStamped;
        cloud_pose_result_->pose = matching_result_->pose;
        matching_result_mutex_.unlock();
        IsPointCloudUpdate = true;
        return true;
    } 

    void GridMapLocalization::Matching() {
        if (matcher_.IsMapReady()) {
            //PreprocessPointCloud(point_cloud_);
            matcher_.SetInitialValue(matching_init_->pose); 
            //matcher_.SetInitialValue(matching_result_->pose);
            matcher_.IcpMatch(point_cloud_);
            matcher_.GetMatchingResult(matching_result_->pose);
            matching_init_->timestamp=matching_result_->timestamp;
            matching_init_->pose=matching_result_->pose;
        }
    }
    void GridMapLocalization::PreprocessPointCloud() {
        // Filter the point cloud using a PassThrough filter.
        util::PositivePassThroughFilterPointCloud(point_cloud_,
                                                  cloud_bounding_box_.min.x,
                                                  cloud_bounding_box_.min.y,
                                                  cloud_bounding_box_.min.z,
                                                  cloud_bounding_box_.max.x,
                                                  cloud_bounding_box_.max.y,
                                                  cloud_bounding_box_.max.z);     
        util::DownsamplePointCloud(point_cloud_,
                                   voxelgrid_filter_size_,
                                   voxelgrid_filter_size_,
                                   voxelgrid_filter_size_);  
        *point_cloud_filtered_=*point_cloud_;
    }
}

