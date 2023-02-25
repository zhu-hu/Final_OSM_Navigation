#!/bin/bash

echo "run..."
source ~/Projects/E100/devel/setup.bash
gnome-terminal --tab --title "planning_bag" -e 'bash -c "rosbag record /localization/estimation /perception/local_grid_map/compressed /perception/global_objects /speed_cmd /e100/speed_feedback /steer_cmd /e100/steer_feedback /e100/vehicle_state /imu/angular_velocity /strong/fix /strong/heading /strong/raw_data /slam/gps/fix /slam/gps/heading /visualization/best_trajectories /visualization/reference_central_lines; exec bash;"'
