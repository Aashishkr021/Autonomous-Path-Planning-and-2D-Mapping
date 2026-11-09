#!/bin/bash
# One-time installs (Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic)
set -e
sudo apt update
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-slam-toolbox ros-jazzy-rviz2 \
     ros-jazzy-rqt-image-view python3-opencv libxml2-utils tmux
pip3 install --user --upgrade --break-system-packages scipy MAVProxy pymavlink ultralytics
python3 -c "import numpy, scipy, ultralytics; from scipy import ndimage; print('python libs OK, numpy', numpy.__version__)"
