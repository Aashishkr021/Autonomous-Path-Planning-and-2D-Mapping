#!/bin/bash
source "$(dirname "$0")/../env.sh"
cd "$NIDAR_HOME/worlds"
gz sim -v4 -r airmouse_maze.sdf
