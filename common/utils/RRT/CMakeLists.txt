cmake_minimum_required(VERSION 2.8.3)
project(rrt)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  cyber_msgs
  dstar_map
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  geometry_msgs
#   ompl
)

find_package(ompl REQUIRED NO_MODULE)
# find_package(glog REQUIRED NO_MODULE)

# include("../../cmake/FindOMPL.cmake")
include("../../cmake/FindGlog.cmake")

find_package(OpenCV)
find_package(Eigen3 REQUIRED NO_MODULE)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Dstar
  CATKIN_DEPENDS cv_bridge image_transport cyber_msgs dstar_map roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ../
)
message("OMPL_INCLUDE_DIRS = ${OMPL_INCLUDE_DIRS}")

set (COMMON_SRCS    ../path_modify.cpp
                    ../vec2d.cpp
                    ../math_utils.cpp
                    )

set (SRCS   src/rrt.cpp
            src/main.cpp
            )

set (SRCS2  src/rrt_choose.cpp
            src/rrt_choose_main.cpp
            ${COMMON_SRCS}
            )

set (SRCS3  src/continue_rrt.cpp
            src/contine_rrt_main.cpp
            )


set (SRCS4  src/continue_rrt_choose.cpp
            src/continue_rrt_choose_main.cpp
            )

set (SRCS5  src/TrajectoryCombiner.cpp)

set (SRCS6  src/GridMapDilate.cpp)

set(COMMON_LIBRARIES 	${catkin_LIBRARIES}
                        ${OpenCV_LIBRARIES}
                        ${OMPL_LIBRARIES}
                        ${GLOG_LIBRARIES}
        )

add_executable(${PROJECT_NAME} ${SRCS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${COMMON_LIBRARIES})


add_executable(rrt_choose ${SRCS2})
add_dependencies(rrt_choose ${catkin_EXPORTED_TARGETS})
target_link_libraries(rrt_choose ${COMMON_LIBRARIES})


add_executable(continue_rrt ${SRCS3})
add_dependencies(continue_rrt ${catkin_EXPORTED_TARGETS})
target_link_libraries(continue_rrt ${COMMON_LIBRARIES})


add_executable(continue_rrt_choose ${SRCS4})
add_dependencies(continue_rrt_choose ${catkin_EXPORTED_TARGETS})
target_link_libraries(continue_rrt_choose ${COMMON_LIBRARIES})

add_executable(combine ${SRCS5})
add_dependencies(combine ${catkin_EXPORTED_TARGETS})
target_link_libraries(combine ${COMMON_LIBRARIES})

add_executable(grid_map_dialte ${SRCS6})
add_dependencies(grid_map_dialte ${catkin_EXPORTED_TARGETS})
target_link_libraries(grid_map_dialte ${COMMON_LIBRARIES})
