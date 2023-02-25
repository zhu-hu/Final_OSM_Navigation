/*
 * @Author: your name
 * @Date: 2021-05-10 12:20:32
 * @LastEditTime: 2021-06-11 21:22:28
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /localization/localization/src/common/map_frame.h
 */
/**
 *map_frame.h
 *brief:transform between gps and map
 *author:Chen Xiaofeng
 *date:20191028
 **/

#ifndef MAP_FRAME_H
#define MAP_FRAME_H
#include <cmath>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

// for qingdao
// const double EARTH_RAD_EQ = 1000 * 6378.137;  // unit: m
// const double OFFSET_X = 10823640;
// const double OFFSET_Y = 3479275;
// const double SCALE = cos(36.0 * M_PI / 180.0);

// for SJTU
// const double EARTH_RAD_EQ = 1000 * 6378.137; //unit: m
// const double OFFSET_X = 11583000;
// const double OFFSET_Y = 3116000;
// const double SCALE = cos(31.03 * M_PI / 180.0);

// for Changshu
// const double EARTH_RAD_EQ = 1000 * 6378.137; //unit: m
// const double OFFSET_X = 11453000;
// const double OFFSET_Y = 3160000;
// const double SCALE = cos(31.59 * M_PI / 180.0); // for Changshu

// for liuzhou
// const double EARTH_RAD_EQ = 1000 * 6378.137; //unit: m
// const double OFFSET_X = 11108000;
// const double OFFSET_Y = 2552000;
// const double SCALE = cos(24.4 * M_PI / 180.0);

class MapFrame {
 public:
  double x;  // east
  double y;  // north

  MapFrame(int region_num, const double x_in = 0.0, const double y_in = 0.0);

  MapFrame(const MapFrame &pos) : x(pos.x), y(pos.y) {} //目前没有使用这个重载

  ~MapFrame() {}

  sensor_msgs::NavSatFix MapFrame2GPS();

  void GPS2MapFrame(const sensor_msgs::NavSatFix &gps);
  double deadReckoning(const double &travel_distance, const double &yaw_before,
                       const double &turn_radian);
  double calcDistance(const MapFrame &another);

 private:
  double EARTH_RAD_EQ_ = 0;
  double OFFSET_X_ = 0;
  double OFFSET_Y_ = 0;
  double SCALE_ = 0;  
};

#endif
