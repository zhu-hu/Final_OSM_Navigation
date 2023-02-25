//
// Created by localization on 11/5/19.
//

#ifndef SRC_MRPT_ICP_MATCHING_H
#define SRC_MRPT_ICP_MATCHING_H

#include <mrpt/slam/CICP.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/maps/COccupancyGridMap2D.h>

#include "common/struct/Pose2D.h"

#include "pcl2mrpt.h"

namespace localization{
    class GridMapMatching{
    public:

        GridMapMatching();

        void SetInitialValue(Pose2D init_value);

        bool IcpMatch(const util::PointCloudTypePtr &cloud);

        void GetMatchingResult(Pose2D &result);

        void SetMap(mrpt::maps::COccupancyGridMap2D *map_input);

        inline bool IsMapReady(){
            return map_ready_flag_;
        }

        inline mrpt::maps::COccupancyGridMap2D GetMap(){
            return map_;
        }

        inline void SetIcpParameter(){
        }

    private:
        // ICP
        mrpt::slam::CICP icp_;
        mrpt::slam::CICP::TReturnInfo icp_return_info_;
        float icp_running_time_;
        mrpt::poses::CPose2D matching_initial_value_;
        mrpt::poses::CPose2D matching_result_;
        mrpt::math::CMatrixDouble33 matching_covariance_;

        mrpt::maps::COccupancyGridMap2D map_;

        bool map_ready_flag_;

    };
}

#endif //SRC_MRPT_ICP_MATCHING_H
