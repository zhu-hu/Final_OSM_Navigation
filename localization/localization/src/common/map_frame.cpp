/*
 * @Author: your name
 * @Date: 2021-05-10 12:20:32
 * @LastEditTime: 2021-06-11 21:29:26
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /localization/localization/src/common/map_frame.cpp
 */
/**
  *map_frame.cpp
  *brief:transform between gps and map
  *author:Chen Xiaofeng
  *date:20191028
  **/

#include "map_frame.h"

MapFrame::MapFrame(int region_num, const double x_in, const double y_in): x(x_in), y(y_in){
    if(region_num == 0){
        EARTH_RAD_EQ_ = 1000 * 6378.137;  // unit: m
        OFFSET_X_ = 10823640;
        OFFSET_Y_ = 3479275;
        SCALE_ = cos(36.0 * M_PI / 180.0);   // for qingdao
    }
    else if(region_num == 1){
        EARTH_RAD_EQ_ = 1000 * 6378.137;  // unit: m
        OFFSET_X_ = 11108000;
        OFFSET_Y_ = 2552000;
        SCALE_ = cos(24.4 * M_PI / 180.0);   // for liuzhou
    }
    else if(region_num == 2){
        EARTH_RAD_EQ_ = 1000 * 6378.137;  // unit: m
        OFFSET_X_ = 11583000;
        OFFSET_Y_ = 3116000;
        SCALE_ = cos(31.03 * M_PI / 180.0);   // for SJTU
    }
    else if(region_num == 3){
        EARTH_RAD_EQ_ = 1000 * 6378.137;  // unit: m
        OFFSET_X_ = 11453000;
        OFFSET_Y_ = 3160000;
        SCALE_ = cos(31.59 * M_PI / 180.0);   // for changshu
    }

}

sensor_msgs::NavSatFix MapFrame::MapFrame2GPS(){
    double lat = 360.0 / M_PI * atan(exp((y+OFFSET_Y_)/(SCALE_ * EARTH_RAD_EQ_)))-90.0;
    double lon = 180.0 * (x + OFFSET_X_) / (SCALE_ * EARTH_RAD_EQ_ * M_PI);
    sensor_msgs::NavSatFix gps;
    gps.latitude = lat;
    gps.longitude = lon;
    return gps;
}
void MapFrame::GPS2MapFrame(const sensor_msgs::NavSatFix &gps){
    x = SCALE_ * EARTH_RAD_EQ_ * gps.longitude * M_PI / 180.0 - OFFSET_X_;
    y = SCALE_ * EARTH_RAD_EQ_ * log(tan((90.0 + gps.latitude) * (M_PI / 360.0))) - OFFSET_Y_;
}
double MapFrame::deadReckoning(const double &travel_distance,const double &yaw_before, const double &turn_radian){
    double yaw = yaw_before + turn_radian/2;
    x += travel_distance * cos(yaw);
    y += travel_distance * sin(yaw);
    yaw += turn_radian/2;
    return yaw;
}
double MapFrame::calcDistance(const MapFrame &another){
    return sqrt(pow(x - another.x,2) + pow(y - another.y,2));
}
