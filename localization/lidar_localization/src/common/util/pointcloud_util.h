//
// Created by zhibo on 9/25/19.
//

#ifndef SRC_POINTCLOUD_UTIL_H
#define SRC_POINTCLOUD_UTIL_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

namespace localization {
    namespace util {

        typedef pcl::PointXYZ PointType;
        typedef pcl::PointCloud<PointType> PointCloudType;
        typedef PointCloudType::Ptr PointCloudTypePtr;

        void PositivePassThroughFilterPointCloud(PointCloudTypePtr &cloud_to_filter,
                                                 double min_x, double min_y, double min_z,
                                                 double max_x, double max_y, double max_z);

        void NegativePassThroughFilterPointCloud(PointCloudTypePtr &cloud_to_filter,
                                                 double min_x, double min_y, double min_z,
                                                 double max_x, double max_y, double max_z);

        void DownsamplePointCloud(PointCloudTypePtr &cloud_to_filter,
                                  double leaf_size_x,
                                  double leaf_size_y,
                                  double leaf_size_z);

        void TransformPointCloud(PointCloudTypePtr &cloud_to_transform,
                                 double pose_x, double pose_y, double pose_z,
                                 double pose_rx, double pose_ry, double pose_rz);

    } // namespace util
} // namespace localization


#endif //SRC_POINTCLOUD_UTIL_H
