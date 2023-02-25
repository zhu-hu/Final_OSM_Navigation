#!/bin/bash

source ../devel/setup.bash

echo "run..."
gnome-terminal --tab --title "roscore" -e 'bash -c "sleep 1; roscore; exec bash;"' \
--tab --title "rviz" -e 'bash -c "roslaunch --wait hdmap_rviz all_display.launch; exec bash;"' \
--tab --title "localization" -e 'bash -c "roslaunch --wait localization imu_fusion.launch; exec bash;"' \
--tab --title "grid_map_perception" -e 'bash -c "roslaunch --wait lidar_perception perception.launch; exec bash;"' \
--tab --title "free_space_peception" -e 'bash -c "roslaunch --wait lidar_perception_free_space perception.launch; exec bash;"' \
--tab --title "planning" -e 'bash -c "roslaunch --wait planning planning.launch; exec bash;"' \
--tab --title "local_control" -e 'bash -c "roslaunch --wait local_steer_controller local_pure_pursuit.launch; exec bash;"' \
--tab --title "osm_map" -e 'bash -c "roslaunch --wait osm_map_rviz osm_map_rviz.launch; exec bash;"' \
