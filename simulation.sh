#!/bin/bash

source ../devel/setup.bash

echo "run..."
gnome-terminal --tab --title "roscore" -e 'bash -c "sleep 1; roscore; exec bash;"' \
--tab --title "rviz" -e 'bash -c "sleep 10; roslaunch hdmap_rviz all_display.launch; exec bash;"' \
--tab --title "planning" -e 'bash -c "sleep 12; roslaunch planning planning.launch; exec bash;"' \
--tab --title "osm_map" -e 'bash -c "sleep 13; roslaunch osm_map_rviz osm_map_rviz.launch; exec bash;"' \
--tab --title "task-ui" -e 'bash -c "sleep 14; rosrun planning planning_ui.py; exec bash;"' \
--tab --title "rosbag" -e 'bash -c "sleep 15; rosparam set use_sim_time false; sleep 1; rosbag play ../bags/2022-05-09/2022-05-09-21-13-32.bag --topics /localization/estimation /perception/local_grid_map /perception/local_grid_map/compressed -l -s 213.0; exec bash;"' \
#--tab --title "tf_base_link2local_map" -e 'bash -c "sleep 6; rosrun tf static_transform_publisher -20.0 20.0 0 0 0 -0.707 0.707 base_link local_map 20; exec bash;"' \
#--tab --title "localization" -e 'bash -c "sleep 3; roslaunch localization imu_fusion.launch; exec bash;"' \
#--tab --title "perception" -e 'bash -c "sleep 3; roslaunch lidar_perception perception.launch; exec bash;"' \/home/perception/Projects/Logistic_Vehicle_Net_Center/bags/2022-05-06/wanglian/2022-05-06-15-53-53.bag
#/home/perception/Projects/Logistic_Vehicle_Net_Center/bags/2022-05-09/2022-05-09-18-27-06.bag
#/home/perception/Projects/Logistic_Vehicle_Net_Center/2022-05-09-20-37-42.bag
#/home/perception/Projects/Logistic_Vehicle_Net_Center/bags/2022-05-09/2022-05-09-21-13-32.bag
#--tab --title "dstar" -e 'bash -c "sleep 12; rosrun dstar dstar; exec bash;"' \
#--tab --title "dstar_map" -e 'bash -c "sleep 13; rosrun dstar_map dstar_map; exec bash;"' \
#--tab --title "rrt" -e 'bash -c "sleep 13; roslaunch rrt all_rrt.launch; exec bash;"' \
#--tab --title "routing" -e 'bash -c "sleep 12; roslaunch routing routing_server.launch; exec bash;"' \
#--tab --title "rosbag" -e 'bash -c "sleep 15; rosparam set use_sim_time false; sleep 1; rosbag play ../bags/2022-05-09/2022-05-09-21-13-32.bag --topics /localization/estimation /perception/local_grid_map /perception/local_grid_map/compressed -l -r 2.0; exec bash;"' \
#/home/perception/Projects/Logistic_Vehicle_Net_Center/bags/2022-05-24/2022-05-24-19-20-31.bag//zhenzhenlou
#/home/perception/Projects/Logistic_Vehicle_Net_Center/bags/2022-05-24/2022-05-24-15-23-24.bag//loop
