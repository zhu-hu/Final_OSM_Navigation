cmake_minimum_required(VERSION 2.8.3)
project(osm_parser)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  message_generation
        std_msgs
        sensor_msgs
        # nav_core
        # pluginlib
        # navfn
        tf
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(TinyXML REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#    FILES
#         newTarget.srv
#         cancelledPoint.srv
#         computeBearing.srv
#     )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#  generate_messages(
#    DEPENDENCIES
#    std_msgs
#    sensor_msgs
#  )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES osm_planner
  CATKIN_DEPENDS roscpp message_runtime
  DEPENDS system_lib TinyXML
)

###########
## Build ##
###########


include_directories(
 include 
 src/ 
 ${catkin_INCLUDE_DIRS} 
 ${TinyXML_INCLUDE_DIRS}
)

set (SRC 
        src/coordinates_converter/haversine_formula.cpp
        # src/coordinates_converter/wgs_84_elipsoid.cpp
        src/path_finder_algorithm/dijkstra.cpp
        src/osm_parser.cpp
)

add_executable(osm_parser_node  
               ${SRC} 
               src/main.cpp
               )
target_link_libraries(osm_parser_node 
                      ${catkin_LIBRARIES} 
                      ${TinyXML_LIBRARIES}
                      )
add_dependencies(osm_parser_node 
                 ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                 ${catkin_EXPORTED_TARGETS}
                 )


# add_library(osm_parser
#         src/osm_parser.cpp
#         src/coordinates_converters/haversine_formula.cpp
#         src/coordinates_converters/wgs_84_elipsoid.cpp
#         )
# target_link_libraries(osm_parser
#         ${catkin_LIBRARIES}
# 	${TinyXML_LIBRARIES}
#         )
# add_dependencies(osm_parser ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ## Declare a C++ library
# add_library(osm_planner
#         src/osm_planner.cpp
#         src/osm_parser.cpp
#         src/osm_localization.cpp
#         src/path_finder_algorithm/dijkstra.cpp
#         src/coordinates_converters/haversine_formula.cpp
#         src/coordinates_converters/wgs_84_elipsoid.cpp

#         #  src/auto_initalization.cpp
#         )
# target_link_libraries(osm_planner
#         ${catkin_LIBRARIES}
# 	${TinyXML_LIBRARIES}
#         )

# add_dependencies(osm_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${TinyXML_LIBRARIES})

# add_executable(osm_planner_node src/osm_planner_node.cpp)
# target_link_libraries(osm_planner_node osm_planner  ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
# add_dependencies(osm_planner_node osm_planner)

# add_executable(navigation_example src/navigation_example.cpp)
# target_link_libraries(navigation_example ${catkin_LIBRARIES})
# add_dependencies(navigation_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(osm_helper src/osm_helper.cpp)
# target_link_libraries(osm_helper ${catkin_LIBRARIES})
# add_dependencies(osm_helper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
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
# install(
#         TARGETS
#         # osm_planner
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_osm_parser.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


#PLUGINLIB_EXPORT_CLASS(osm::Planner, nav_core::BaseGlobalPlanner)
# install(FILES osm_planner_plugin.xml
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
