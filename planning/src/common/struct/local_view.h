//
// Created by luyifan on 19-11-5.
//

#ifndef PLANNING_LOCAL_VIEW_H
#define PLANNING_LOCAL_VIEW_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cyber_msgs/LaneWaypoint.h"
#include "cyber_msgs/LocalizationEstimate.h"
#include "prediction_obstacles.h"
#include "routing_response.h"
#include "traffic_light_detection.h"

namespace planning {
struct LocalView {
  const std::vector<PredictionObstacle> *prediction_obstacles;
  //原始栅格图
  const cv::Mat *grid_map;
  //膨胀腐蚀之后的栅格图
  const cv::Mat *dilated_grid_map;
  // freespace的栅格图
  const cv::Mat *fs_grid_map;
  const cyber_msgs::LocalizationEstimate *localization_estimate;
  const TrafficLightDetection *traffic_light;
  const std::vector<cyber_msgs::LaneWaypoint> *routing_sequence;
  bool nudge = true;
  int nudge_trajectory_num = 3;  //奇数,最低为3
  double nudge_width = 2.8;
};
}  // namespace planning

#endif  // PLANNING_LOCAL_VIEW_H
