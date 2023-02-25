//
// Created by localization on 10/29/19.
//地图更新

#ifndef SRC_GRID_MAP_2D_H
#define SRC_GRID_MAP_2D_H

#include <mrpt/maps/COccupancyGridMap2D.h>

#include "common/util/str_util.h"
#include "common/util/time_util.h"

#include <omp.h>

namespace localization {

    class GridMap2D {
    public:
        GridMap2D();

        ~GridMap2D(){
            delete map;
        };

        void InitGridMap(int map_size,
                         double map_resolution,
                         std::string map_folder_path);

        bool UpdateGridMap(float position_x, float position_y);

        void LoadGridMap(const int center_x, const int center_y);

        int GetSquareCenterCoordinate(float coordinate);

    private:

        bool IsCenterChanged(int &map_center_x, int &map_center_y,
                             float position_x, float position_y);


    public:
        mrpt::maps::COccupancyGridMap2D *map;

    private:

        int map_center_x_;
        int map_center_y_;
        int map_size_;
        unsigned square_size_;
        double map_resolution_;
        std::string map_folder_path_;

    };
}


#endif //SRC_GRID_MAP_2D_H
