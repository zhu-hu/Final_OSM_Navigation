/*
 * Cobjectyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao
 * Tong University. All rights reserved. Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.03.10
 */

#ifndef SENSOR_FUSION_SENSOR_FUSION_H
#define SENSOR_FUSION_SENSOR_FUSION_H

#include <cyber_msgs/ObjectArray.h>
#include <mrpt/poses/CPose3D.h>

#include <vector>

#include "common/object.h"
#include "common/util.h"
#include "object_segmentation/object_segmenter.h"
#include "sensor_fusion/data_association.h"
#include "sensor_fusion/kalman_filter.h"

class SensorFusion {
 public:
  SensorFusion();

  void ObjectCallback(const cyber_msgs::ObjectArray& meas_objects,
                      double time_stamp, cyber_msgs::ObjectArray& out_objects);

 private:
  int id_max_ = 1000;
  int cur_id_ = 0;

  std::vector<TrackingObject> track_objects_;

  DataAssociation data_association_;

  KalmanFilter kf_;

 private:
  void OutputObject(cyber_msgs::ObjectArray& out_objects);
};

#endif  // SENSOR_FUSION_SENSOR_FUSION_H
