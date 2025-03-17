#!/bin/bash

# Source the ROS environment
source /opt/ros/noetic/setup.bash

# Source the catkin workspace
source /catkin_ws/devel/setup.bash

# Execute the command passed to the container
exec "$@"
