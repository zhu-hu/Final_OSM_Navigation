//
// Created by zhou on 9/30/19.
//

#include "pointcloud_util.h"

namespace localization {
    namespace util {

        void PositivePassThroughFilterPointCloud(PointCloudTypePtr &cloud_to_filter,
                                                 double min_x, double min_y, double min_z,
                                                 double max_x, double max_y, double max_z) {
            pcl::PassThrough<PointType> pass;
            pass.setInputCloud(cloud_to_filter);

            pass.setFilterFieldName("x");
            pass.setFilterLimits(min_x, max_x);
            pass.filter(*cloud_to_filter);

            pass.setFilterFieldName("y");
            pass.setFilterLimits(min_y, max_y);
            pass.filter(*cloud_to_filter);

            pass.setFilterFieldName("z");
            pass.setFilterLimits(min_z, max_z);
            pass.filter(*cloud_to_filter);
        }

        void NegativePassThroughFilterPointCloud(PointCloudTypePtr &cloud_to_filter,
                                                 double min_x, double min_y, double min_z,
                                                 double max_x, double max_y, double max_z) {
            
            PointCloudTypePtr cloud_filtered(new PointCloudType());

            int cloud_size = cloud_to_filter->points.size();
            for (int i = 0; i < cloud_size; ++i) {
                float x = cloud_to_filter->points[i].x;
                float y = cloud_to_filter->points[i].y;
                float z = cloud_to_filter->points[i].z;
                if (x < max_x && x > min_x && y < max_y && y > min_y && z < max_z && z > min_z) continue;

                cloud_filtered->push_back(cloud_to_filter->points[i]);
            }
            cloud_to_filter->clear();
            cloud_to_filter = cloud_filtered;

        }

        void DownsamplePointCloud(PointCloudTypePtr &cloud_to_filter,
                                  double leaf_size_x,
                                  double leaf_size_y,
                                  double leaf_size_z) {
            pcl::VoxelGrid<PointType> sor;
            PointCloudTypePtr cloud_filtered(new PointCloudType());
            sor.setInputCloud(cloud_to_filter);
            sor.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
            sor.filter(*cloud_filtered);

            cloud_to_filter = cloud_filtered;
        }

        void TransformPointCloud(PointCloudTypePtr &cloud_to_transform,
                                 double pose_x, double pose_y, double pose_z,
                                 double pose_rx, double pose_ry, double pose_rz) {
            PointCloudTypePtr cloud_to_be_transformed(cloud_to_transform);
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            transform.translation() << pose_x, pose_y, pose_z;
            transform.rotate(Eigen::AngleAxisf(pose_rz, Eigen::Vector3f::UnitZ()) *
                             Eigen::AngleAxisf(pose_ry, Eigen::Vector3f::UnitY()) *
                             Eigen::AngleAxisf(pose_rx, Eigen::Vector3f::UnitX()));

            PointCloudTypePtr transformed_cloud(new PointCloudType());
            pcl::transformPointCloud(*cloud_to_be_transformed, *cloud_to_transform, transform);
        }

    }
}
