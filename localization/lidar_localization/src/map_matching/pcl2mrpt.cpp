//
// Created by localization on 11/5/19.
//

#include "pcl2mrpt.h"

namespace localization {

        void pcl2mrpt(const util::PointCloudTypePtr &pcl_cloud,
                      mrpt::maps::CSimplePointsMap &mrpt_cloud) {
            int cloud_size = pcl_cloud->points.size();
            for (int i = 0; i < cloud_size; ++i) {
                mrpt_cloud.insertPoint(pcl_cloud->points[i].x,
                                       pcl_cloud->points[i].y,
                                       pcl_cloud->points[i].z);
            }
        }
} // namespace localization
