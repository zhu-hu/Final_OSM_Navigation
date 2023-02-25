//
// Created by localization on 11/5/19.
//

#ifndef SRC_PCL2MRPT_H
#define SRC_PCL2MRPT_H

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CPointsMap.h>

#include "common/util/pointcloud_util.h"

namespace localization {

        void pcl2mrpt(const util::PointCloudTypePtr &pcl_cloud, mrpt::maps::CSimplePointsMap &mrpt_cloud);

} // namespace localization

#endif //SRC_PCL2MRPT_H
