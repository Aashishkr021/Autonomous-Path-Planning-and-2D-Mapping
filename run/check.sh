#!/bin/bash
# quick health check while everything is running
source "$(dirname "$0")/../env.sh"
echo "--- gazebo topics"; gz topic -l | grep -E "scan|camera|iris"
echo "--- ROS topics";    ros2 topic list | grep -E "scan|odom|tf|map|camera"
echo "--- TF chain";      timeout 5 ros2 run tf2_ros tf2_echo map laser | head -8
echo "--- clock publishers (must be 1)"; ros2 topic info /clock -v | grep -i "publisher count"
