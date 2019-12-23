#!/bin/bash
set -e

# source environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/catkin_ws/devel/setup.bash"
exec "$@"
