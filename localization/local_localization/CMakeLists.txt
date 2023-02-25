cmake_minimum_required(VERSION 3.5)
project(local)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  cyber_msgs
  tf
)

find_package(Eigen3)

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp rospy
)

###########
## Build ##
###########

include_directories(
    src
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

#add_library(map_frame_lib src/common/map_frame.cpp)
#add_library(ekf_pose_lib src/ekf/ekf_pose.cpp)

add_executable(local 
               src/main.cpp src/top.cpp 
               #src/common/map_frame.cpp 
               src/ekf/ekf_pose.cpp 
)
add_dependencies(local ${catkin_EXPORTED_TARGETS})
target_link_libraries(local ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

