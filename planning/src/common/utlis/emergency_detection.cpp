#include "emergency_detection.h"

namespace planning {

void emergency_detect(const cv::Mat &grid_map, uint32_t &emergency_mode,
                      int &emergency_time_cnt_, int &close_time_cnt_,
                      const Parameter *param) {
  bool need_stop = false;

  int emergency_cnt = 0;
  int close_cnt = 0;

  const double length = param->behavior_param_.vehicle_length;  //车长
  const double width = param->behavior_param_.vehicle_width;    //车宽
  const double rear_to_back =
      param->behavior_param_.rear_to_back;  //后轴到车尾的距离
  const double rear_to_front = length - rear_to_back;

  const uint32_t kLocalGridMapHeight =
      param->grid_map_param_.max_x -
      param->grid_map_param_.min_x;  // 栅格图高度
  const uint32_t kLocalGridMapWidth =
      param->grid_map_param_.max_y -
      param->grid_map_param_.min_y;  // 栅格图宽度
  const uint32_t kPixelScale =
      param->grid_map_param_.pixel_scale;  // 10个栅格为1米
  const uint32_t kCarCenterX =
      kLocalGridMapWidth / 2 * kPixelScale;  // 栅格图车辆中心X坐标
  const uint32_t kCarCenterY =
      param->grid_map_param_.max_x * kPixelScale;  // 栅格图车辆中心Y坐标

  // 紧急制动区参数(单位为栅格图格数)
  // 紧急制动区宽度
  const int kEmergencyHalfWidth =
      param->behavior_param_.emergency_stop_area_width / 2 * kPixelScale;
  // 紧急制动区高度
  const int kEmergencyHeight =
      param->behavior_param_.emergency_stop_area_height * kPixelScale;
  // 前车长高度
  const int kCarHeight = 3 * kPixelScale;
  // 紧急制动区左上角X坐标
  const int kEmergencyUpLeftX = kCarCenterX - kEmergencyHalfWidth;
  // 紧急制动区右下角X坐标
  const int kEmergencyDownRightX = kCarCenterX + kEmergencyHalfWidth;
  // 紧急制动区左上角Y坐标
  const int kEmergencyUpLeftY =
      kCarCenterY - kEmergencyHeight - rear_to_front * kPixelScale;
  // const int kEmergencyDownRightY = kCarCenterY - kCarHeight - rear_to_front;
  // 紧急制动区右下角Y坐标
  const int kEmergencyDownRightY = kCarCenterY - rear_to_front * kPixelScale;
  // 紧急制动区障碍栅格数阈值
  const int emergency_cnt_threshold =
      param->behavior_param_.emergency_stop_grid_count_threshold;
  // 紧急制动区时间阈值
  const int emergency_time_cnt_threshold =
      param->behavior_param_.emergency_stop_time_count_threshold;

  // 减速区参数(单位为栅格图格数)
  // 减速区宽度
  const int kCloseHalfWidth =
      param->behavior_param_.emergency_speed_down_area_width / 2 * kPixelScale;
  // 减速区高度
  const int kCloseHeight =
      param->behavior_param_.emergency_speed_down_area_height * kPixelScale;

  // 减速区左上角X坐标
  const int kCloseUpLeftX = kCarCenterX - kCloseHalfWidth;
  // 减速区右下角X坐标
  const int kCloseDownRightX = kCarCenterX + kCloseHalfWidth;
  // 减速区左上角Y坐标
  const int kCloseUpLeftY =
      kCarCenterY - kCloseHeight - rear_to_front * kPixelScale;
  // const int kCloseDownRightY = kCarCenterY - kCarHeight - rear_to_front; //
  // 减速区右下角Y坐标
  const int kCloseDownRightY = kCarCenterY - rear_to_front * kPixelScale;
  // 减速区障碍栅格数阈值
  const int close_cnt_threshold =
      param->behavior_param_.emergency_speed_down_grid_count_threshold;
  // 减速区时间阈值
  const int close_time_cnt_threshold =
      param->behavior_param_.emergency_speed_down_time_count_threshold;

  // 保护区参数(单位为栅格图格数)
  // 保护区宽度
  const int kProtectHalfWidth = 
      param->behavior_param_.protect_area_width / 2 * kPixelScale;
  // 保护区高度
  const int kProtectHeight = param->behavior_param_.protect_area_height * kPixelScale;
  // 刹停区左上角X坐标
  const int kProtectUpLeftX = kCarCenterX - kProtectHalfWidth;
  // 刹停区右下角X坐标
  const int kProtectDownRightX = kCarCenterX + kProtectHalfWidth;
  // 刹停区左上角Y坐标
  const int kProtectUpLeftY =
      kCarCenterY - kProtectHeight - rear_to_front * kPixelScale;
  // 刹停区右下角Y坐标
  const int kProtectDownRightY = kCarCenterY - rear_to_front * kPixelScale;

  for (int i = kCloseUpLeftX; i < kCloseDownRightX; ++i) {
    for (int j = kCloseUpLeftY; j < kCloseDownRightY; ++j) {
      if (grid_map.at<uchar>(j, i) == 255) {
        // std::cout << "colliding in close area!" << std::endl;
        close_cnt++;
        if (i >= kEmergencyUpLeftX && i < kEmergencyDownRightX &&
            j >= kEmergencyUpLeftY && j < kEmergencyDownRightY) {
          // std::cout << "colliding in emergency area!" << std::endl;
          emergency_cnt++;
        }
        if (i >= kProtectUpLeftX && i < kProtectDownRightX && 
            j >= kProtectUpLeftY && j < kProtectDownRightY) {
          // std::cout << "colliding in emergency area!" << std::endl;
          need_stop = true;
          break;
        }
      }
      // grid_map_copy.at<uchar>(j, i) = 255;
    }
  }

  // cv::Mat grid_map_copy(grid_map.size(), CV_8UC1, cv::Scalar::all(0));
  // grid_map_copy.at<uchar>(kEmergencyUpLeftY, kEmergencyUpLeftX) = 255;
  // grid_map_copy.at<uchar>(kEmergencyDownRightY, kEmergencyDownRightX) = 255;
  // grid_map_copy.at<uchar>(kCloseUpLeftY, kCloseUpLeftX) = 255;
  // grid_map_copy.at<uchar>(kCloseDownRightY, kCloseDownRightX) = 255;
  // cv::imshow("grid", grid_map_copy);
  // cv::waitKey(1);

  if (need_stop) {
    emergency_mode = 2;
    return;
  }

  if (emergency_cnt > emergency_cnt_threshold) {
    if (emergency_time_cnt_ < emergency_cnt_threshold * 2)
      emergency_time_cnt_++;
  } else {
    if (emergency_time_cnt_ > 0) emergency_time_cnt_--;
  }

  if (close_cnt > close_cnt_threshold) {
    if (close_time_cnt_ < close_cnt_threshold * 2) close_time_cnt_++;
  } else {
    if (close_time_cnt_ > 0) close_time_cnt_--;
  }

  if (emergency_time_cnt_ >= emergency_time_cnt_threshold)
    emergency_mode = 2;
  else {
    if (close_time_cnt_ >= close_time_cnt_threshold)
      emergency_mode = 1;
    else
      emergency_mode = 0;
  }
}

}  // namespace planning
