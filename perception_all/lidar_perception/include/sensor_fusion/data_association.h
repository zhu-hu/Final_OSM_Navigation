/*
 * Cobjectyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao
 * Tong University. All rights reserved. Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.03.10
 */

#ifndef SENSOR_FUSION_DATA_ASSOCIATION_H
#define SENSOR_FUSION_DATA_ASSOCIATION_H

#include <cyber_msgs/ObjectArray.h>
#include <mrpt/poses/CPose3D.h>

#include <algorithm>
#include <iostream>
#include <vector>

#include "common/object.h"
#include "common/util.h"

class DataAssociation {
 public:
  // greedy algorithm for data association
  std::vector<std::pair<std::vector<int>, std::vector<int>>>
  GreedyAlgorithmLidar(const cyber_msgs::ObjectArray& meas_objects,
                       const std::vector<TrackingObject>& track_objects);

 private:
  float size_weight_ = 0.4;
  float lidar_threshold_ = 0.7;

 private:
  // calculate association value between tracked object and measured object
  float AssociationValueTrackedLidar(const cyber_msgs::Object& meas_object,
                                     const TrackingObject& track_object);
};

#endif  // SENSOR_FUSION_DATA_ASSOCIATION_H
