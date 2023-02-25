//
// Created by huyao on 2018/6/10.
// Email: hooyao@sjtu.edu.cn
//

#ifndef PROJECT_MAP_PARAM_H
#define PROJECT_MAP_PARAM_H

#include <cmath>

namespace map_param {

//    constexpr double kOriginX = 443723.467;
//    constexpr double kOriginY = 4403288.508;
    constexpr double kOriginX = 350438.044;
    constexpr double kOriginY = 3433648.568;

    namespace grid_map{

        constexpr double kCellSize = 0.2;
        constexpr double kCellFactor = 1/kCellSize;

        constexpr int kHeight = 600;
        constexpr int kWidth = 250;
        constexpr int kCarCenterX = 125;//栅格图左下角为原点
        constexpr int kCarCenterY = 100;

        constexpr int kCarRadius = 6;

        constexpr int kCarCenterXCVIndex = kCarCenterX;
        constexpr int kCarCenterYCVIndex = kHeight - kCarCenterY;

        constexpr double kInitVarThreshold = 250;
        constexpr int kObstacleThreshold = 254;
        constexpr int kDDilateSize = 11;
        constexpr int kDDilateDeltaSize = 2;

        constexpr double kHeightResolution = 0.01176 ;
        constexpr double kRampThreshold = 0.018     ;// 0.4663  ;
        constexpr double kRampThreshold2 = 0.05     ;// 0.4663  ;
    }

    namespace dstar{
        constexpr double extend_factor = 1.5;
        constexpr int min_global_map_size = 100 * grid_map::kCellFactor;
        constexpr int local_map_margin = 20;
        constexpr int local_map_margin_x = 20;
        constexpr int local_map_margin_top = 250;
        constexpr int local_map_margin_bottom = 100;
        constexpr double goal_length = 15.0;
        constexpr double start_margin = 0.5;
        constexpr double goal_margin = 80.0;
    }

    namespace attr1{
        const int START		        = 0;
        const int ENTRANCE  		= 1;
        const int INTERSECTION_EXIT	= 2;
        const int COMMON	    	= 3;
        const int PARK_ENTRANCE		= 4;
        const int PARK_EXIT	    	= 5;
        const int PARK			    = 6;
        const int END		    	= 7;
        const int INTERPOLATION		= 8;
        const int COUNTRY_ROAD      = 14;
        const int ROAD_CHECK        = 21;
        const int TUNNEL            = 10;
    }

   namespace attr2{
        const int UNKNOWN 		= 0;
        const int GO_STRAIGHT 		= 1;
        const int TURN_RIGHT		= 2;
        const int TURN_LEFT 		= 3;
        const int U_TURN 		= 4;
        const int TRAFFIC_SIGN 		= 5;
        const int SLOW_DOWN 		= 6;
        const int NO_CHANGE_LANE    = 7;
    }
}

#endif //PROJECT_MAP_PARAM_H
