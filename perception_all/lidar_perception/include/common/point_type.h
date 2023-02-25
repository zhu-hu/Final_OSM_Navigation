/*
 * Copyright (C) 2020 by CyberC3 Intelligent Vehicle Labs, Shanghai Jiao Tong
 * University. All rights reserved.
 * Zhang W <zhangw6338@sjtu.edu.cn>
 * @ created at 2020.02.20
 */

#ifndef COMMON_POINT_TYPE_H_
#define COMMON_POINT_TYPE_H_

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointTypeCloud;
typedef PointTypeCloud::Ptr PointTypeCloudPtr;
typedef PointTypeCloud::ConstPtr PointTypeCloudConstPtr;

#endif  // COMMON_POINT_TYPE_H_
