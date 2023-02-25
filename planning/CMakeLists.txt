cmake_minimum_required(VERSION 3.5)
project(planning)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
cmake_modules
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        visualization_msgs
        cyber_msgs
        cyber_srvs
        image_transport
        cv_bridge
        tf)

find_package(OpenCV)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(ompl REQUIRED NO_MODULE)
find_package(TinyXML REQUIRED)

# include("cmake/FindOMPL.cmake")
include("cmake/FindGlog.cmake")
include("cmake/FindGflags.cmake")

# find_package(ompl REQUIRED NO_MODULE)
# find_package(glog REQUIRED NO_MODULE)
# find_package(gflags REQUIRED NO_MODULE)

catkin_package(
        DEPENDS system_lib TinyXML
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
        ${OMPL_INCLUDE_DIRS}
        ${GFLAGS_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${TinyXML_INCLUDE_DIRS}
        src/
        src/common/math/
        ../
	../map/
        ../common/utils/
        ../map/hdmap/src/
        src/planner/hybrid_a_star
        ../osm_parser/src/
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/statemachine.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

set (SRCS   src/planning.cpp
            src/common/struct/reference_line.cpp
            src/common/struct/trajectory.cpp
            src/common/struct/frame.cpp
            src/manager/publisher_manager.cpp
            src/manager/self_state_manager.cpp
            src/manager/grid_map_manager.cpp
            src/manager/osm_map_manager.cpp
            src/manager/osm_route_manager.cpp
            src/state/StPause.cpp
            src/state/StWaitForRun.cpp
            src/state/StRun.cc
            src/state/StStart.cc
            src/state/StNavigation.cc
            src/common/optimal_jmt/optimal_jmt.cpp
            src/planner/planner_factory.cpp
            src/planner/dubins_planner.cpp
            src/planner/dubins_curve.cpp
            src/common/utlis/emergency_detection.cpp
            src/common/utlis/optimal_reference_line.cpp
            src/common/utlis/collision_detection.cpp
            )


set (MAP_SRCS
           ../map/hdmap/src/json_adapter/json_adapter.cpp
           ../map/hdmap/src/json_adapter/junctions_json_parser.cpp
           ../map/hdmap/src/json_adapter/lanes_json_parser.cpp
           ../map/hdmap/src/json_adapter/roads_json_parser.cpp
           ../map/hdmap/src/json_adapter/overlaps_json_parser.cpp
           ../map/hdmap/src/json_adapter/signals_json_parser.cpp
           ../map/hdmap/src/json_adapter/stop_signs_json_parser.cpp
           ../map/hdmap/src/json_adapter/util_json_parser.cpp
           ../map/hdmap/src/math/AABox2d.cpp
           ../map/hdmap/src/math/box2d.cpp
           ../map/hdmap/src/math/linear_interpolation.cpp
           ../map/hdmap/src/math/LineSegment2d.cpp
           ../map/hdmap/src/math/Polygon.cpp
           ../map/hdmap/src/math/utlis.cpp
           ../map/hdmap/src/math/vec2d.cpp
           ../map/hdmap/src/Impl.cpp
           ../map/hdmap/src/ObjetcInfo.cpp
)

set (OSM_MAP_SRCS
           ../osm_parser/src/coordinates_converter/haversine_formula.cpp
           ../osm_parser/src/path_finder_algorithm/dijkstra.cpp
           ../osm_parser/src/osm_parser.cpp
)


set (SMOOTHER_SRCS
        src/common/math/discrete_points_math.cc
        src/common/math/discrete_points_referenceline_smoother.cpp
        src/common/math/fem_pos_deviation_osqp_interface.cc
        src/common/math/fem_pos_deviation_smoother.cc
)

add_executable(${PROJECT_NAME}_node 
               ${SRCS} 
               ${MAP_SRCS} 
               ${OSM_MAP_SRCS}
               ${SMOOTHER_SRCS}
               src/main.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#message(${OMPL_LIBRARIES})

set(COMMON_LIBRARIES 	${catkin_LIBRARIES}
                        ${OpenCV_LIBRARIES}
                        ${OMPL_LIBRARIES}
                        ${GFLAGS_LIBRARIES}
                        ${GLOG_LIBRARIES}
                        ${TinyXML_LIBRARIES}
                        jsoncpp
                        osqp)
## Specify libraries to link a library or executable target against
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${COMMON_LIBRARIES} math_utils geometry)

#set(CMAKE_CXX_FLAGS   "-lboost_system")

add_executable(dubins_curve_test src/planner/dubins_curve.cpp
                                 src/planner/dubins_test_main.cpp)
add_dependencies(dubins_curve_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(dubins_curve_test ${COMMON_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)