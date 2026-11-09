#!/bin/bash
source "$(dirname "$0")/../env.sh"
python3 "$NIDAR_HOME/scripts/record_video.py" "${1:-/camera/detections_image}"
