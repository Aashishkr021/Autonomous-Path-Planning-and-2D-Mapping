#!/bin/bash
source "$(dirname "$0")/../env.sh"
cd "$NIDAR_HOME"
python3 "$NIDAR_HOME/scripts/yolo_person.py" --ros-args -p use_sim_time:=true "$@"