#!/bin/bash
pkill -f parameter_bridge; pkill -f slam_toolbox; pkill -f static_transform_publisher; pkill -f rviz2
pkill -f auto_explore; pkill -f yolo_person; pkill -f arducopter; pkill -f mavproxy; pkill -f "gz sim"
echo stopped
