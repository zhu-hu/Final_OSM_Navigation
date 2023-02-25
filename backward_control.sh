#!/bin/bash

source ../devel/setup.bash

echo "run..."
gnome-terminal --tab --title "roscore" -e 'bash -c "sleep 1; roscore; exec bash;"' \
--tab --title "urg_driver" -e 'bash -c "roslaunch --wait urg_node urg_lidar.launch; exec bash;"' \
--tab --title "localization" -e 'bash -c "roslaunch --wait localization imu_fusion.launch; exec bash;"' \
--tab --title "link_angle_estimator" -e 'bash -c "roslaunch --wait link_angle_estimator link_angle_estimator.launch; exec bash;"' \
--tab --title "path_pub" -e 'bash -c "roslaunch --wait pplqr_back_controller path_pub.launch; exec bash;"' \
--tab --title "pplqr_back_control" -e 'bash -c "roslaunch --wait pplqr_back_controller pplqr_back_controller.launch; exec bash;"' \
