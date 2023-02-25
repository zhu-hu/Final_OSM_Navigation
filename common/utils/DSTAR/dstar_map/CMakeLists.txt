cmake_minimum_required(VERSION 2.8.3)
project(dstar_map)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        cv_bridge
        geometry_msgs
        image_transport
        cyber_msgs
        cyber_srvs
        roscpp
        rospy
        sensor_msgs
        std_msgs
        tf
        )

add_message_files(
        FILES
        CellUpdateList.msg
        CellUpdate.msg
        WayPoint.msg
        WayPointList.msg
        StampedFloat64.msg
)

add_service_files(
        FILES
        MapUpdate.srv
        RoadCheckGoal.srv
)

generate_messages(
        DEPENDENCIES
        cyber_msgs
        std_msgs
        sensor_msgs
        geometry_msgs
        dstar_map
)

find_package(OpenCV)

catkin_package(
        CATKIN_DEPENDS message_runtime roscpp std_msgs cyber_msgs cyber_srvs geometry_msgs sensor_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        include ${roscpp_INCLUDE_DIRS}
        ../../
)

set (SRCS   src/CreateMap.cpp
        src/main.cpp)

set(COMMON_LIBRARIES 	${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES})
message("OpenCV_LIBRARIES = ${OpenCV_LIBRARIES}")

add_executable(${PROJECT_NAME} ${SRCS})
add_dependencies(${PROJECT_NAME}  ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${COMMON_LIBRARIES})

