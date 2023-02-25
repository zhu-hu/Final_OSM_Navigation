#ifndef LIDAR_PREPROCESS_LIDAR_PREPROCESS_H_
#define LIDAR_PREPROCESS_LIDAR_PREPROCESS_H_

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Eigen>
#include <queue>
struct LidarPreprocessParams {
    // roi filter params(base_link coordinate)
    double near_noise_x_min = -1.25;
    double near_noise_x_max = 2.30;
    double near_noise_y_min = -0.40;
    double near_noise_y_max = 0.40;
    double livox_roi_x_min = -10.0;
    double livox_roi_x_max = 30.0;
    double livox_roi_y_min = -10.0;
    double livox_roi_y_max = 10.0;
    double livox_roi_z_min = -0.50;
    double livox_roi_z_max = 1.50;
    // remove close noise
    bool livox_noise_flag = false;
    double livox_noise_x_min = -5.00;
    double livox_noise_x_max = 10.00;
    double livox_noise_y_min = -6.00;
    double livox_noise_y_max = 6.00;
    double livox_noise_rate = 0.0015;
    double min_direct_remove_intensity = 1.0;
    // remove trailer noise
    double trailer_x_min = -1.70;
    double trailer_y_min = -1.30;
    double trailer_y_max = 1.30;
    // remove outlier
    double min_outlier_intensity = 5.5;
    double search_radius = 1.0;
    int min_neighbors = 5;
    // overlap area
    double angle_up_right = 21;          //单位 角度
    double angle_up_left = 34;           //单位 角度
    double angle_down_right = 82;        //单位 角度
    double angle_tan_up_right = 0.3838;  // tan(angle_up_right)
    double angle_tan_up_left = 0.6745;
    double angle_tan_down_right = 7.1154;
};
class LidarPreprocess {
public:
    LidarPreprocess(const LidarPreprocessParams &params);
    void CloudPreprocess(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                         pcl::PointCloud<pcl::PointXYZI> &cloud_out);

private:
    LidarPreprocessParams params_;
};

#endif  // LIDAR_PREPROCESS_LIDAR_PREPROCESS_H_