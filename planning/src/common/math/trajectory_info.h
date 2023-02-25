#ifndef PLANNING_TRAJECTORY_INFO_H
#define PLANNING_TRAJECTORY_INFO_H

#include "common/struct/trajectory.h"
#include "math.h"

namespace planning{
    inline void calculate_trajectory_info(Trajectory* trajectory){
        auto& points = trajectory->mutable_points();
        if (points.size() < 2)
            return;
        //计算heading
        for(size_t i = 0; i < points.size() - 1; i++){
            points[i].theta = atan2(points[i+1].y - points[i].y, points[i+1].x - points[i].x);
        }
        points.back().theta = points[points.size() - 2].theta;
        // points[0].theta = atan2(points[1].y - points[0].y, points[1].x - points[0].x);
        // for(size_t i = 1; i < points.size() - 1; i++){
        //     points[i].theta = atan2(points[i+1].y - points[i-1].y, points[i+1].x - points[i-1].x);
        // }
        // points.back().theta = points[points.size() - 2].theta;

        //计算kappa
        for(size_t i = 0; i < points.size() - 1; i++){
            double heading_diff = normalize_angle(points[i+1].theta - points[i].theta);
            double distance = sqrt(pow(points[i+1].x - points[i].x, 2.0) + pow(points[i+1].y - points[i].y, 2.0));
            if (distance <= 0)
                continue;
            double kappa = fabs(heading_diff) / distance;
            points[i].kappa = kappa;
        }
        points.back().kappa = points[points.size() - 2].kappa;
    }
}//planning


#endif