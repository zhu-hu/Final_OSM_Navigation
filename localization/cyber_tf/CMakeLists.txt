cmake_minimum_required(VERSION 2.8.3)
project(cyber_tf)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  cyber_msgs
  cyber_srvs
  geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tf_car
#  CATKIN_DEPENDS roscpp rospy tf tiggo_msgs tiggo_srvs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

set (SRCS src/main.cpp)

set(LIBRARIES 	${catkin_LIBRARIES})

add_executable(${PROJECT_NAME} ${SRCS})
target_link_libraries(${PROJECT_NAME} ${LIBRARIES})
