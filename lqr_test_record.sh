#!/bin/bash

source ../devel/setup.bash

echo "run..."
gnome-terminal --tab --title "roscore" -e 'bash -c "sleep 1; roscore; exec bash;"' \
--tab --title "lqr_test_record" -e 'bash -c "sleep 2; rosbag record /localization/estimation /e100/steer_feedback /fusion_angle_degree /desired_beta_degree /steer_cmd /speed_cmd; exec bash;"' \
