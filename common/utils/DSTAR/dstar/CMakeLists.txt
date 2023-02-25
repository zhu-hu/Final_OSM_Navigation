cmake_minimum_required(VERSION 2.8.3)
project(dstar)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  cyber_msgs
  image_transport
  dstar_map
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  geometry_msgs
)

find_package(SBPL REQUIRED sbpl)
find_package(OpenCV)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Dstar
  CATKIN_DEPENDS cv_bridge image_transport cyber_msgs dstar_map roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SBPL_INCLUDE_DIRS}
  ../../
)

set (SRCS   src/Dstar.cpp
            src/main.cpp)

set(COMMON_LIBRARIES 	${catkin_LIBRARIES}
                        ${OpenCV_LIBRARIES}
                        ${SBPL_LIBRARIES})

add_executable(${PROJECT_NAME} ${SRCS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${COMMON_LIBRARIES})
