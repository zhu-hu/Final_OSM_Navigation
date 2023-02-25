#!/bin/bash

source ../devel/setup.bash

echo "run..."
gnome-terminal --tab --title "roscore" -e 'bash -c "roscore; ec bash;"' \
--tab --title "urg_driver" -e 'bash -c "roslaunch --wait urg_node urg_lidar.launch; exec bash;"' \
--tab --title "localization" -e 'bash -c "roslaunch --wait localization imu_fusion.launch; exec bash;"' \
--tab --title "grid_map_perception" -e 'bash -c "roslaunch --wait lidar_perception perception.launch; exec bash;"' \
--tab --title "free_space_peception" -e 'bash -c "roslaunch --wait lidar_perception_free_space perception.launch; exec bash;"' \
--tab --title "edge_perception" -e 'bash -c "roslaunch --wait collision_detection run.launch; exec bash;"' \
--tab --title "ekf_estimator" -e 'bash -c "roslaunch --wait link_angle_estimator link_angle_estimator.launch; exec bash;"' \
--tab --title "echo gps status" -e 'bash -c "sleep 10; rostopic echo /strong/raw_data; exec bash;"' \
#--tab --title "rosbag_record" -e 'bash -c "sleep 5; cd bags; rosbag record /localization/estimation /perception/local_grid_map /perception/local_grid_map/compressed; exec bash;
#--tab --title "tf_base_link2local_map" -e 'bash -c "sleep 6; rosrun tf static_transform_publisher -20.0 20.0 0 0 0 -0.707 0.707 base_link local_map 20; exec bash;"' \
#--tab --title "driver" -e 'bash -c "roslaunch --wait e100bringup driver_with_hub.launch; exec bash;"' \

