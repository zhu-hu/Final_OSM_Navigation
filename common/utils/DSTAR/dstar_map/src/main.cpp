//
// Created by luyifan on 18-8-20.
//

#include "create_map/CreateMap.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "DSMap");
    ros::NodeHandle pnh("~");
    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;

    CreateMap map_creator(&pnh, &listener, &broadcaster);
    std::cout<<"Wait for goal..."<<std::endl;

    ros::waitForShutdown();
}