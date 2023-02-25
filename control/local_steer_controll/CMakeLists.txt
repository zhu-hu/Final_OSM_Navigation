cmake_minimum_required(VERSION 2.8.3)
project(local_steer_controller)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	dynamic_reconfigure 
	message_generation 
	roscpp 
	std_msgs
    sensor_msgs	
    tf
    cyber_msgs
)

#add_message_files(
#  FILES
##  NodeExampleData.msg
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  tiggo_msgs
#)

# generate_dynamic_reconfigure_options(
#   cfg/stanley_param.cfg
#   cfg/pursuit_param.cfg
# )

catkin_package(
  	CATKIN_DEPENDS 
  	dynamic_reconfigure 
  	message_runtime 
  	roscpp 
  	std_msgs
	sensor_msgs
	cyber_msgs
)

include_directories(
	include ${catkin_INCLUDE_DIRS}
)

# Set the name to use for the executable.
set (BINNAME1 local_pure_pursuit)


# Set the source files to use with the executable.
set (SRCS1  src/local_pure_pursuit.cpp)

#Build the executable that will be used to run this node.
add_executable (${BINNAME1} 
                ${SRCS1} 
)
add_dependencies(${BINNAME1}  ${catkin_EXPORTED_TARGETS})
target_link_libraries(${BINNAME1} ${catkin_LIBRARIES})

# install(TARGETS ${BINNAME1}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(DIRECTORY
#   launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )