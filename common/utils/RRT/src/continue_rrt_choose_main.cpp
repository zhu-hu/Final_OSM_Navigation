//
// Created by huyao on 18-8-24.
//

#include "rrt/crrt_choose.h"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "rrt_choose");
    ros::NodeHandle pnh("~");

    tf::TransformListener *listener = new tf::TransformListener();

    huyao_RRT::CRRTChoose my_crrt_choose(&pnh, listener);

    ros::spin();

    return 0;
}