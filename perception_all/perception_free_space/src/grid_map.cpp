/*
 * @Author: Liuyaqi99 283402121@qq.com
 * @Date: 2022-09-26 11:34:46
 * @LastEditors: Liuyaqi99 283402121@qq.com
 * @LastEditTime: 2022-09-26 12:58:30
 * @FilePath: /zh_ws/src/perception/perception_free_space/src/grid_map.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "grid_map.h"
namespace perception {
void GridMap::setParams(const GridMapParams &params) {
  params_ = params;
  // roi grid map
  roi_map_height_ =
      static_cast<int>((params_.roi_params.max_x - params_.roi_params.min_x) *
                       params_.roi_params.pixel_scale);
  roi_map_width_ =
      static_cast<int>((params_.roi_params.max_y - params_.roi_params.min_y) *
                       params_.roi_params.pixel_scale);
  roi_map_height_origin_ = static_cast<int>(params_.roi_params.max_x *
                                            params_.roi_params.pixel_scale);
  roi_map_width_origin_ = static_cast<int>(params_.roi_params.max_y *
                                           params_.roi_params.pixel_scale);
}

void GridMap::generate_grid_map(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                                cv::Mat &roi_grid_map) {
  roi_grid_map =
      cv::Mat(roi_map_height_, roi_map_width_, CV_8UC1, cv::Scalar::all(255));
  for (int i = 0; i < cloud_in.size(); ++i) {
    int col, row;
    const pcl::PointXYZI &pt = cloud_in.points[i];
    LocalToPixel(pt.x, pt.y, row, col);
    if (row >= 0 && row < roi_map_height_ && col >= 0 && col < roi_map_width_) {
      roi_grid_map.at<uchar>(row, col) = (uchar)0;
    }
  }
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(roi_grid_map, roi_grid_map, cv::MORPH_OPEN, kernel);
}

void GridMap::LocalToPixel(const float &real_x, const float &real_y,
                           int &pixel_row, int &pixel_coln) {
  pixel_row = roi_map_height_origin_ -
              static_cast<int>(real_x * params_.roi_params.pixel_scale);
  pixel_coln = roi_map_width_origin_ -
               static_cast<int>(real_y * params_.roi_params.pixel_scale);
}

void GridMap::PixelToLocal(const float &pixel_row, const float &pixel_coln,
                           float &real_x, float &real_y) {
  real_x =
      (roi_map_height_origin_ - pixel_row) / params_.roi_params.pixel_scale;
  real_y =
      (roi_map_width_origin_ - pixel_coln) / params_.roi_params.pixel_scale;
}

} // namespace perception