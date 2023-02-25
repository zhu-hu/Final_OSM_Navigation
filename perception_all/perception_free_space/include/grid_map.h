/*
 * @Author: Liuyaqi99 283402121@qq.com
 * @Date: 2022-06-28 15:41:00
 * @LastEditors: Liuyaqi99 283402121@qq.com
 * @LastEditTime: 2022-06-28 16:01:49
 * @FilePath: /free_space_detect/src/livox_free_space/include/grid_map.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef GRID_MAP_GRID_MAP_H
#define GRID_MAP_GRID_MAP_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/common/common.h>
#include <ros/ros.h>
namespace perception {

struct RoiMapParams {
  double min_x = -10.0;
  double max_x = 30.0;
  double min_y = -10.0;
  double max_y = 10.0;
  int pixel_scale = 10;
}; // struct RoiMapParams

struct GridMapParams {
  RoiMapParams roi_params;
}; // GridMapParams

class GridMap {
public:
  GridMapParams params_;

  // roi grid map
  int roi_map_height_;
  int roi_map_width_;

  void generate_grid_map(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                         cv::Mat &roi_grid_map);

  void setParams(const GridMapParams &params);

private:
  // roi grid map
  int roi_map_height_origin_;
  int roi_map_width_origin_;

  // coordination transform: real to pixel
  void LocalToPixel(const float &real_x, const float &real_y, int &pixel_row,
                    int &pixel_column);

  // coordination transform: pixel to real
  void PixelToLocal(const float &pixel_row, const float &pixel_column,
                    float &real_x, float &real_y);
};
} // namespace perception
#endif // GRID_MAP_GRID_MAP_H