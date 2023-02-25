/*
 * @Author: your name
 * @Date: 2021-01-11 21:36:46
 * @LastEditTime: 2021-06-08 13:28:53
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /localization_v2_ws/src/lidar_localiztion/localization/lidar_localization/src/map_matching/mrpt_icp_matching.cpp
 */
//
// Created by localization on 11/5/19.
//

#include "mrpt_icp_matching.h"

namespace localization {

    GridMapMatching::GridMapMatching() {
        icp_.options.maxIterations = 25;
        icp_.options.skip_quality_calculation = true;
        icp_.options.skip_cov_calculation = true;

//        icp_.options.minAbsStep_trans = 1e-3;
//        icp_.options.minAbsStep_rot = 1e-3;
//        icp_.options.thresholdDist = 1;
//        icp_.options.ALFA = 0.5;
//        icp_.options.smallestThresholdDist = 0.1;
//        icp_.options.onlyClosestCorrespondences = true;
        icp_.options.corresponding_points_decimation = 12;

        map_ready_flag_ = false;
    }

    void GridMapMatching::SetInitialValue(Pose2D init_value) {
        matching_initial_value_.x() = init_value.x;
        matching_initial_value_.y() = init_value.y;
        matching_initial_value_.phi() = init_value.phi;
    }

    bool GridMapMatching::IcpMatch(const localization::util::PointCloudTypePtr &cloud) {
        mrpt::maps::CSimplePointsMap cloud_to_match;
        localization::pcl2mrpt(cloud, cloud_to_match);

        // Map matching.
        mrpt::poses::CPosePDFPtr pdf = icp_.AlignPDF(
                &map_,
                &cloud_to_match,
                mrpt::poses::CPosePDFGaussian(matching_initial_value_),
                &icp_running_time_,
                (void *) &icp_return_info_);

        pdf->getCovarianceAndMean(matching_covariance_, matching_result_);
        pdf->getCovariance(matching_covariance_);

    }

    void GridMapMatching::GetMatchingResult(Pose2D &result) {
        result.x = matching_result_.x();
        result.y = matching_result_.y();
        result.phi = matching_result_.phi();
    }

    void GridMapMatching::SetMap(mrpt::maps::COccupancyGridMap2D *map_input) {
        map_ = *map_input;
        map_ready_flag_ = true;
    }

} // namespace localization
