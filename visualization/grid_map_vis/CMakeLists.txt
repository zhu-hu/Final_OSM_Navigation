cmake_minimum_required(VERSION 2.8.3)
project(grid_map_vis)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cyber_msgs
  tf
)

find_package(OpenCV)
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include ${roscpp_INCLUDE_DIRS}
)

set(COMMON_LIBRARIES 	${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES})

set (USB_KEY_SRCS
    ../../common/Pwd_8/syunew3.cpp
    ../../common/Pwd_8/libusb/core.c
    ../../common/Pwd_8/libusb/descriptor.c
    ../../common/Pwd_8/libusb/io.c
    ../../common/Pwd_8/libusb/sync.c
    ../../common/Pwd_8/libusb/os/linux_usbfs.c
)

add_executable(${PROJECT_NAME} 
               src/main.cpp
               # ${USB_KEY_SRCS}
)
add_dependencies(${PROJECT_NAME}  ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${COMMON_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)