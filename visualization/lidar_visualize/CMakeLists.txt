cmake_minimum_required(VERSION 2.8.3)
project(lidar_visualize)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cyber_msgs
  roscpp
  visualization_msgs
  tf
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_executable(global_visualize_node src/global_visualize.cpp)

## Add cmake target dependencies of the library
add_dependencies(global_visualize_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(global_visualize_node ${catkin_LIBRARIES})

install(TARGETS global_visualize_node
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