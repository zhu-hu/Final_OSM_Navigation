//
// Created by localization on 10/29/19.
//

#include "grid_map_2d.h"

using namespace localization::util;

namespace localization {

    GridMap2D::GridMap2D() {
        map = nullptr;
    }

    void GridMap2D::InitGridMap(int map_size, double map_resolution, std::string map_folder_path) {
        map_center_x_ = 999999;
        map_center_y_ = 999999;
        map_size_ = map_size;
        square_size_ = 50;
        map_resolution_ = map_resolution;
        map_folder_path_ = map_folder_path;
    }
    //判断中心是否改变，更改地图区域
    bool GridMap2D::UpdateGridMap(float position_x, float position_y) {
        if (IsCenterChanged(map_center_x_, map_center_y_,
                            position_x, position_y)) {
            LoadGridMap(map_center_x_,map_center_y_);
            return true;
        } else {
            return false;
        }
    }

    //地图加载用于mrpt匹配，参数一为要输出的地图
    void GridMap2D::LoadGridMap(const int center_x, const int center_y) {
        // 读以center为中心的n×n的方形,此时的center已做对齐处理
        float x_min = center_x - float(square_size_) / 2 * float(map_size_);
        float x_max = center_x + float(square_size_) / 2 * float(map_size_);
        float y_min = center_y - float(square_size_) / 2 * float(map_size_);
        float y_max = center_y + float(square_size_) / 2 * float(map_size_);
        int image_size_ = int(float(square_size_) / map_resolution_);
        mrpt::maps::COccupancyGridMap2D *ogm = new mrpt::maps::COccupancyGridMap2D;

        ogm->clear();
        ogm->setSize(x_min, x_max, y_min, y_max, map_resolution_);

        int image_around_num = 0;
        clock_t map_loading_start_time = clock();

//#pragma omp parallel for
        for (int i = 0; i < map_size_; ++i) {
//  //          #pragma omp parallel for
            for (int j = 0; j < map_size_; ++j) {
                //每张图片的中心
                int new_center_x = center_x + square_size_ * (i - map_size_ / 2);
                int new_center_y = center_y + square_size_ * (j - map_size_ / 2);
                // int to string
                std::string str_x = Int2FixedStringWithSign(new_center_x, 7);
                std::string str_y = Int2FixedStringWithSign(new_center_y, 7);
                std::string file_name = "cloud_" + str_x + "_" + str_y + "_0000000.png";

                std::string file_path = map_folder_path_ + file_name;

                mrpt::utils::CImage image_in;

                if (!image_in.loadFromFile(file_path, 0)) {
//#pragma omp parallel for 如果没找到图片，在单元格涂白色
                    for (int x = 0; x < image_size_; ++x) {
                        //#pragma omp parallel for
                        for (int y = 0; y < image_size_; ++y) {
                            int square_index_x = x + i * image_size_;
                            int square_index_y = y + j * image_size_;
                            ogm->setCell(square_index_x, square_index_y, 0.99999); 
                        }
                    }
                } else {
//#pragma omp parallel for 如果找到图片，在单元格图与图片对应颜色
                    for (int x = 0; x < image_size_; ++x) {
                        //#pragma omp parallel for
                        for (int y = 0; y < image_size_; ++y) {
                            int square_index_x = x + i * image_size_;
                            int square_index_y = y + j * image_size_;
                            ogm->setCell(square_index_x, square_index_y,
                                        image_in.getAsFloat(x, image_size_ - 1 - y));
                        }
                    }
                    image_around_num++;
                }
            }
        }
        //??
        mrpt::maps::COccupancyGridMap2D *temp_map = map;
        map = ogm;
        delete temp_map;

        std::cout << "[INFO] Map loading time: " << GetTimeInterval(map_loading_start_time) * 1000 << " ms."
                  << std::endl; // Map loading timer ends, unit s->ms
        //搜索的图片不足加载的一半
        if (image_around_num < map_size_ * map_size_ / 2) {
            std::cout << "[WARN] Not enough map image data." << std::endl;
        }
    }
    bool GridMap2D::IsCenterChanged(int &map_center_x, int &map_center_y, float position_x, float position_y) {

        int map_center_x_new = GetSquareCenterCoordinate(position_x);
        int map_center_y_new = GetSquareCenterCoordinate(position_y);

        if ((map_center_x != map_center_x_new) ||
            (map_center_y != map_center_y_new)) {
            map_center_x = map_center_x_new;
            map_center_y = map_center_y_new;
            std::cout << "[INFO] Map center changed to (" << map_center_x << "," << map_center_y << ")."
                      << std::endl;
            return true;
        } else {
            return false;
        }
    }

    int GridMap2D::GetSquareCenterCoordinate(float coordinate) {
        return square_size_ * ceil((float(coordinate) - float(square_size_) / 2) / float(square_size_));
    }

}