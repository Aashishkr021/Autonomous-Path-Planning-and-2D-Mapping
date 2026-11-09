#!/bin/bash
source "$(dirname "$0")/../env.sh"
python3 "$NIDAR_HOME/scripts/odom_from_ekf.py" --ros-args -p use_sim_time:=true
