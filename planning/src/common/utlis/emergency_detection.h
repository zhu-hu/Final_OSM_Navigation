#ifndef EMERGENCY_CONTROL_H
#define EMERGENCY_CONTROL_H

#include <ros/ros.h>

#include <std_msgs/Int8.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include "common/params/vehicle_model.h"
#include "parameter/Parameter.h"

namespace planning
{

void emergency_detect(const cv::Mat &grid_map, uint32_t &emergency_mode, int &emergency_time_cnt_, int &close_time_cnt_, const Parameter *param);

} // namespace planning

#endif