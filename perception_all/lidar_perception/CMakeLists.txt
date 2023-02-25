cmake_minimum_required(VERSION 2.8.3)
project(lidar_perception)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
  cv_bridge
  image_transport
  cyber_msgs
)
# MRPT
find_package(MRPT 1.5 REQUIRED)
# OpenCV
find_package(OpenCV REQUIRED)

## The catkin_package macro generates cmake config files for your package
catkin_package(
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set (USB_KEY_SRCS
    ../common/Pwd_8/syunew3.cpp
    ../common/Pwd_8/libusb/core.c
    ../common/Pwd_8/libusb/descriptor.c
    ../common/Pwd_8/libusb/io.c
    ../common/Pwd_8/libusb/sync.c
    ../common/Pwd_8/libusb/os/linux_usbfs.c
)

## Declare a C++ executable
add_executable(lidar_perception_node
  src/node.cpp
  src/common/util.cpp
  src/lidar_preprocess/lidar_preprocess.cpp
  src/ground_segmentation/line_fitting.cpp
  src/grid_map/grid_map.cpp
  src/object_segmentation/object_segmenter.cpp
  src/sensor_fusion/data_association.cpp
  src/sensor_fusion/kalman_filter.cpp
  src/sensor_fusion/sensor_fusion.cpp
  # ${USB_KEY_SRCS}
)

add_executable(near_safe
  src/near_safe.cpp
  src/common/util.cpp
  src/lidar_preprocess/lidar_preprocess.cpp
  src/ground_segmentation/line_fitting.cpp
  src/grid_map/grid_map.cpp
  # ${USB_KEY_SRCS}
)

add_dependencies(lidar_perception_node ${catkin_EXPORTED_TARGETS})
add_dependencies(near_safe ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(lidar_perception_node ${catkin_LIBRARIES} ${MRPT_LIBS} ${OpenCV_LIBRARIES})
target_link_libraries(near_safe ${catkin_LIBRARIES} ${MRPT_LIBS} ${OpenCV_LIBRARIES})

install(TARGETS lidar_perception_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS near_safe
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
