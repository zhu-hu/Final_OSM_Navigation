cmake_minimum_required(VERSION 3.5)
project(localization)

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

link_directories(lib/)

set (USB_KEY_SRCS
    ../../common/Pwd_8/syunew3.cpp
    ../../common/Pwd_8/libusb/core.c
    ../../common/Pwd_8/libusb/descriptor.c
    ../../common/Pwd_8/libusb/io.c
    ../../common/Pwd_8/libusb/sync.c
    ../../common/Pwd_8/libusb/os/linux_usbfs.c
)

add_library(map_frame_lib src/common/map_frame.cpp)
add_library(ekf_pose_lib src/ekf/ekf_pose.cpp)

add_executable(localization 
               src/main.cpp src/top.cpp 
               src/common/map_frame.cpp 
               src/ekf/ekf_pose.cpp 
               # ${USB_KEY_SRCS}
)
add_dependencies(localization ${catkin_EXPORTED_TARGETS})
target_link_libraries(localization ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

install(TARGETS localization
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY
  rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

