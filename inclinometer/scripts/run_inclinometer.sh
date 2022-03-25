#!/bin/bash
cd "$(dirname "$0")"
source /opt/ros/$ROS_DISTRO/setup.bash
source /catkin_ws/devel/setup.bash
set -e

roslaunch inclinometer inclinometer_with_rviz.launch