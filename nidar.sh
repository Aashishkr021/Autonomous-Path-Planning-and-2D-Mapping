#!/bin/bash
# ONE command to start everything:   ./nidar.sh start | attach | stop | check
# Uses tmux: one window per program, so you can see every log in a single terminal.
source "$(dirname "$(readlink -f "$0")")/env.sh"
S=nidar
case "$1" in
  start)
    tmux kill-session -t $S 2>/dev/null
    "$NIDAR_HOME/run/stop_all.sh" >/dev/null 2>&1; sleep 2
    tmux new-session -d -s $S -n gazebo "$NIDAR_HOME/run/1_gazebo.sh"
    sleep 15
    tmux new-window -t $S -n sitl   "$NIDAR_HOME/run/2_sitl.sh"
    echo "Wait ~60 s for ArduPilot to finish starting (STABILIZE> in the 'sitl' window)..."; sleep 60
    tmux new-window -t $S -n mapping "$NIDAR_HOME/run/3_mapping.sh"
    sleep 15
    tmux new-window -t $S -n yolo      "$NIDAR_HOME/run/5_yolo.sh"
    tmux new-window -t $S -n survivors "$NIDAR_HOME/run/5b_survivors.sh"
    tmux new-window -t $S -n explore "$NIDAR_HOME/run/4_explore.sh"
    echo "Started. Watch with:  ./nidar.sh attach   (Ctrl+B then 0-3 switches window, Ctrl+B then D leaves)"
    ;;
  yolo)   tmux new-window -t $S -n yolo   "$NIDAR_HOME/run/5_yolo.sh" ;;
  record) tmux new-window -t $S -n record "$NIDAR_HOME/run/6_record.sh" ;;
  attach) tmux attach -t $S ;;
  check)  "$NIDAR_HOME/run/check.sh" ;;
  stop)   tmux kill-session -t $S 2>/dev/null; "$NIDAR_HOME/run/stop_all.sh" ;;
  *) echo "usage: $0 start | yolo | record | attach | check | stop" ;;
esac
