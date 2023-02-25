#!/bin/bash

source ../devel/setup.bash

echo "run..."
gnome-terminal --tab --title "roscore" -e 'bash -c "sleep 1; roscore; exec bash;"' \
--tab --title "rosbag_record" -e 'bash -c "sleep 2; cd bags; rosbag record /fusion_angle_degree /livox_measure_angle_degree /laser_measure_angle_degree /link_angle_degree /localization/estimation /perception/local_grid_map /perception/local_grid_map/compressed /e100/speed_feedback /e100/steer_feedback /imu/angular_velocity /strong/fix /strong/heading /strong/raw_data /tf /desired_beta_degree; exec bash;"' \
#--tab --title "tf_base_link2local_map" -e 'bash -c "sleep 6; rosrun tf static_transform_publisher -20.0 20.0 0 0 0 -0.707 0.707 base_link local_map 20; exec bash;"' \
