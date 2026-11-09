#!/bin/bash
source "$(dirname "$0")/../env.sh"
mkdir -p "$NIDAR_HOME/maps"
python3 "$NIDAR_HOME/scripts/survivor_tagger.py" --ros-args -p use_sim_time:=true -p out_prefix:="$NIDAR_HOME/maps/survivors" "$@"