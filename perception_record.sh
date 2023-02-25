#!/bin/bash

echo "run..."
source ~/Projects/E100/devel/setup.bash
gnome-terminal --tab --title "perception_bag" -e 'bash -c "rosbag record /localization/estimation /driver/livox/point_cloud; exec bash;"'
