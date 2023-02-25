#!/bin/bash

source ../devel/setup.bash

echo "run..."
gnome-terminal --tab --title "roscore" -e 'bash -c "sleep 1; roscore; exec bash;"' \
--tab --title "rviz" -e 'bash -c "roslaunch --wait hdmap_rviz all_display.launch; exec bash;"' \
--tab --title "grid_map_perception" -e 'bash -c "roslaunch --wait lidar_perception perception.launch; exec bash;"' \
--tab --title "free_space_peception" -e 'bash -c "roslaunch --wait lidar_perception_free_space perception.launch; exec bash;"' \
--tab --title "edge_perception" -e 'bash -c "roslaunch --wait collision_detection run.launch; exec bash;"' \
--tab --title "planning" -e 'bash -c "roslaunch --wait planning planning.launch; exec bash;"' \
--tab --title "local_control" -e 'bash -c "roslaunch --wait local_steer_controller local_pure_pursuit.launch; exec bash;"' \
--tab --title "osm_map" -e 'bash -c "roslaunch --wait osm_map_rviz osm_map_rviz.launch; exec bash;"' \
--tab --title "rosbag" -e 'bash -c "sleep 10; rosparam set use_sim_time false; sleep 1; rosbag play ../bags/2023-02-01/zhenzhenlou2.bag --topics /localization/estimation /driver/livox/point_cloud /e100/speed_feedback /e100/steer_feedback /imu/angular_velocity -l; exec bash;"' \
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
#/home/perception/Projects/Logistic_Vehicle_Net_Center/bags/2022-05-31/2022-05-31-17-16-15.bag
#/home/perception/Projects/Logistic_Vehicle_Net_Center/bags/2022-05-31/2022-05-31-15-33-41.bag
#/strong/fix /strong/heading /strong/raw_data
#--tab --title "task-ui" -e 'bash -c "sleep 5; rosrun planning planning_ui.py; exec bash;"' \2022-06-15-18-38-31.bag
#../bags/2022-06-04/2022-06-04-18-57-12.bag
#/perception/local_grid_map /perception/local_grid_map/compressed /perception/freespace_grid_map/compressed
#--tab --title "local_steer_controller" -e 'bash -c "roslaunch --wait local_steer_controller pure_pursuit.launch; exec bash;"' \
