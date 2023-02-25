#include "lidar_preprocess.h"
LidarPreprocess::LidarPreprocess(const LidarPreprocessParams &params) {
    params_ = params;
    params_.angle_tan_up_right = tan(params_.angle_up_right * M_PI / 180.0);
    params_.angle_tan_up_left = tan(params_.angle_up_left * M_PI / 180.0);
    params_.angle_tan_down_right = tan(params_.angle_down_right * M_PI / 180.0);
}

void LidarPreprocess::CloudPreprocess(
  const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
  pcl::PointCloud<pcl::PointXYZI> &cloud_out) {
    std::vector<int> indices;
    size_t point_size = cloud_in.size();
    for (size_t i = 0; i < point_size; i++) {
        const pcl::PointXYZI &cur_pt = cloud_in.points[i];
        if (cur_pt.x > params_.livox_roi_x_min
            && cur_pt.x < params_.livox_roi_x_max
            && cur_pt.y > params_.livox_roi_y_min
            && cur_pt.y < params_.livox_roi_y_max
            && cur_pt.z < params_.livox_roi_z_max
            && !(cur_pt.x > params_.near_noise_x_min
                 && cur_pt.x < params_.near_noise_x_max
                 && cur_pt.y > params_.near_noise_y_min
                 && cur_pt.y < params_.near_noise_y_max)
            && !(cur_pt.x > params_.trailer_x_min
                 && cur_pt.x < params_.near_noise_x_min
                 && cur_pt.y > params_.trailer_y_min
                 && cur_pt.y < params_.trailer_y_max)) {
            float tan = cur_pt.y / cur_pt.x;  //去除重叠区域
            if (((tan >= params_.angle_tan_up_right
                  && tan <= params_.angle_tan_up_left)
                 || tan >= params_.angle_tan_down_right)
                || ((-tan >= params_.angle_tan_up_right
                     && -tan <= params_.angle_tan_up_left)
                    || -tan >= params_.angle_tan_down_right)) {
                continue;
            }
            if (params_.livox_noise_flag && cur_pt.x > params_.livox_noise_x_min
                && cur_pt.x < params_.livox_noise_x_max
                && cur_pt.y > params_.livox_noise_y_min
                && cur_pt.y < params_.livox_noise_y_max) {
                if (cur_pt.intensity < params_.min_direct_remove_intensity) {
                    continue;
                }
                const pcl::PointXYZI &prev_pt =
                  cloud_in.points[(i - 1) % point_size];
                const pcl::PointXYZI &next_pt =
                  cloud_in.points[(i + 1) % point_size];
                float prev_delta_x = prev_pt.x - cur_pt.x;
                float prev_delta_y = prev_pt.y - cur_pt.y;
                float prev_distance =
                  prev_delta_x * prev_delta_x + prev_delta_y * prev_delta_y;
                float next_delta_x = next_pt.x - cur_pt.x;
                float next_delta_y = next_pt.y - cur_pt.y;
                float next_distance =
                  next_delta_x * next_delta_x + next_delta_y * next_delta_y;
                float threshold = params_.livox_noise_rate
                                  * (cur_pt.x * cur_pt.x + cur_pt.y * cur_pt.y);
                if (prev_distance < threshold && next_distance < threshold)
                    indices.push_back(i);
            } else {
                indices.push_back(i);
            }
        }
    }

    std::vector<int> final_indices;
    if (params_.livox_noise_flag) {
        size_t remain_size = indices.size();
        for (size_t i = 0; i < remain_size; i++) {
            int cur_index = indices[i];
            const pcl::PointXYZI &cur_pt = cloud_in.points[cur_index];
            if (cur_pt.x > params_.livox_noise_x_min
                && cur_pt.x < params_.livox_noise_x_max
                && cur_pt.y > params_.livox_noise_y_min
                && cur_pt.y < params_.livox_noise_y_max) {
                int prev_index = indices[(i - 1) % remain_size];
                int next_index = indices[(i + 1) % remain_size];
                const pcl::PointXYZI &prev_pt = cloud_in.points[prev_index];
                const pcl::PointXYZI &next_pt = cloud_in.points[next_index];
                float prev_delta_x = prev_pt.x - cur_pt.x;
                float prev_delta_y = prev_pt.y - cur_pt.y;
                float prev_distance =
                  prev_delta_x * prev_delta_x + prev_delta_y * prev_delta_y;
                float next_delta_x = next_pt.x - cur_pt.x;
                float next_delta_y = next_pt.y - cur_pt.y;
                float next_distance =
                  next_delta_x * next_delta_x + next_delta_y * next_delta_y;
                float threshold = params_.livox_noise_rate
                                  * (cur_pt.x * cur_pt.x + cur_pt.y * cur_pt.y);
                if (prev_distance < threshold && next_distance < threshold)
                    final_indices.push_back(cur_index);
            } else {
                final_indices.push_back(cur_index);
            }
        }
    }

    if (params_.livox_noise_flag) {
        cloud_out.resize(indices.size());
        for (size_t i = 0; i < final_indices.size(); i++) {
            cloud_out.points[i] = cloud_in.points[final_indices[i]];
        }
    } else {
        cloud_out.resize(indices.size());
        for (size_t i = 0; i < indices.size(); i++) {
            cloud_out.points[i] = cloud_in.points[indices[i]];
        }
    }
}
