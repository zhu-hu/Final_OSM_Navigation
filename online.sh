#!/bin/bash

source ../devel/setup.bash

echo "run..."
gnome-terminal --tab --title "roscore" -e 'bash -c "sleep 1; roscore; exec bash;"' \
--tab --title "driver" -e 'bash -c "sleep 3; roslaunch e100bringup driver_with_hub.launch; exec bash;"' \
--tab --title "rviz" -e 'bash -c "sleep 3; roslaunch hdmap_rviz all_display.launch; exec bash;"' \
--tab --title "localization" -e 'bash -c "sleep 3; roslaunch localization imu_fusion.launch; exec bash;"' \
--tab --title "perception" -e 'bash -c "sleep 3; roslaunch lidar_perception perception.launch; exec bash;"' \
--tab --title "routing" -e 'bash -c "sleep 5; roslaunch routing routing_server.launch; exec bash;"' \
--tab --title "planning" -e 'bash -c "sleep 6; roslaunch planning planning.launch; exec bash;"' \
--tab --title "tf_base_link2local_map" -e 'bash -c "sleep 6; rosrun tf static_transform_publisher -20.0 20.0 0 0 0 -0.707 0.707 base_link local_map 20; exec bash;"' \
--tab --title "dstar" -e 'bash -c "sleep 6; rosrun dstar dstar; exec bash;"' \
--tab --title "dstar_map" -e 'bash -c "sleep 6; rosrun dstar_map dstar_map; exec bash;"' \
--tab --title "rrt" -e 'bash -c "sleep 6; roslaunch rrt all_rrt.launch; exec bash;"' \
--tab --title "task-ui" -e 'bash -c "sleep 6; rosrun planning planning_ui.py; exec bash;"' \
--tab --title "steer_control" -e 'bash -c "sleep 7; roslaunch steer_controller pure_pursuit.launch; exec bash;"' \
--tab --title "speed_control" -e 'bash -c "sleep 8; roslaunch speed_controller speed_controller.launch; exec bash;"' \

