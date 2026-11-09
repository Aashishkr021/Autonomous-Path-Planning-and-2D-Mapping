#!/bin/bash
source "$(dirname "$0")/../env.sh"
mkdir -p "$NIDAR_HOME/maps"
python3 "$NIDAR_HOME/scripts/auto_explore.py" --ros-args -p map_prefix:="$NIDAR_HOME/maps/airmouse_map" "$@"
