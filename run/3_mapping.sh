#!/bin/bash
source "$(dirname "$0")/../env.sh"
ros2 launch "$NIDAR_HOME/launch/mapping.launch.py" "$@"
