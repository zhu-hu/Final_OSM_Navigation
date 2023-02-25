cmake_minimum_required(VERSION 3.0.2)
project(configs)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES params
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY
  common/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
)

install(DIRECTORY
  control/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/control
)

install(DIRECTORY
  localization/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/localization
)

install(DIRECTORY
  perception/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/perception
)

install(DIRECTORY
  planning/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/planning
)