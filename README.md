# nidar_airmouse

Autonomous indoor mapping + survivor detection in Gazebo (ArduPilot SITL, ROS 2 Jazzy, Gazebo Harmonic).

## Folder layout
```
nidar_airmouse/
├── README.md
├── env.sh                  environment (sourced by every run script)
├── setup.sh                one-time installs
├── nidar.sh                one-command start/stop (tmux)
├── worlds/airmouse_maze.sdf
├── models/nidar_iris_lidar/ drone: lidar + camera + odometry (model.sdf, model.config)
├── config/                 bridge.yaml (Gazebo<->ROS), slam.yaml
├── launch/mapping.launch.py   bridge + base_link->laser TF + SLAM + RViz
├── rviz/airmouse.rviz      ground-station layout (map, scan, path, camera)
├── scripts/                auto_explore.py, yolo_person.py, record_video.py, keyboard_fly.py
├── run/                    1_gazebo ... 6_record, check.sh, stop_all.sh
├── maps/                   saved maps (.pgm/.yaml)
├── videos/                 recordings
├── logs/
└── docs/REQUIREMENTS.md    brief requirements vs. status
```

## Install
```bash
cd ~ && unzip nidar_airmouse.zip && cd nidar_airmouse
chmod +x setup.sh run/*.sh scripts/*.py
./setup.sh
```
Needs ~/ardupilot and ~/ardupilot_gazebo already built.

## Entry / exit behaviour (auto_explore.py)
The drone starts OUTSIDE the arena on the launch pad, facing the entrance. After take-off it:
1. ENTER: flies straight ahead `entry_forward_m` (2.0 m) through the entrance,
2. maps only inside the arena (never plans behind `gate_m` = 1.0 m from the start),
3. EXPLORE: flies to unexplored map edges; rooms are detected and remembered as it goes,
4. SWEEP: visits every remembered room it has not been inside, in a short tour order,
5. RETURN/HOME: plans back out through the entrance, lands (LAND mode) on the start point, saves the map.
Disable the room sweep with `-p sweep_rooms:=false`.
If your start pose or entrance changes: `-p entry_forward_m:=2.5 -p gate_m:=1.2`; open area: `-p keep_inside:=false`.
The drone MUST be placed facing the entrance - the entry leg follows its start heading.

## Quick start (one command)
```bash
cd ~/nidar_airmouse && ./setup.sh        # once
./nidar.sh start                         # Gazebo -> SITL -> SLAM+RViz -> explorer (about 2 min to launch)
./nidar.sh attach                        # see all logs (Ctrl+B then D to leave)
./nidar.sh yolo                          # optional person detection
./nidar.sh stop                          # stop everything
```
Map is saved to `maps/airmouse_map.pgm/.yaml`.

## Manual run (one terminal each, in this order)
```bash
~/nidar_airmouse/run/1_gazebo.sh      # Gazebo + world + drone
~/nidar_airmouse/run/2_sitl.sh        # ArduPilot; wait for the STABILIZE> prompt (+ about 1 min)
~/nidar_airmouse/run/3_mapping.sh     # bridge + SLAM + RViz
~/nidar_airmouse/run/check.sh         # optional: all lines should show data
~/nidar_airmouse/run/4_explore.sh     # takes off, explores, returns, lands, saves maps/airmouse_map.*
~/nidar_airmouse/run/5_yolo.sh        # optional: person detection (RViz shows /camera/detections_image)
~/nidar_airmouse/run/6_record.sh      # optional: record video to videos/flight.mp4
```
Stop everything: `run/stop_all.sh`. Ctrl+C in the explorer lands the drone and saves the map.
Do not run keyboard_fly.py at the same time as auto_explore.py.
