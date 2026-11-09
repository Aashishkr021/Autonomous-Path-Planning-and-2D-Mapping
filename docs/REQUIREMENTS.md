# NIDAR AirMouse - requirement status

Source: *Mission Brief - NIDAR AirMouse*. "Sim" = works in the Gazebo setup. "TODO" = not built yet.

| # | Brief requirement | Status | Where |
|---|---|---|---|
| 1 | Autonomous navigation, GPS-denied | Sim (uses simulator ground-truth odometry) | scripts/auto_explore.py |
| 2 | Obstacle avoidance in corridors/rooms | Sim (tested offline on this world) | scripts/auto_explore.py |
| 3 | Live 2D map while flying | Sim | config/slam.yaml, rviz/airmouse.rviz |
| 4 | Live camera feed | Sim | models/.../model.sdf, config/bridge.yaml |
| 5 | Detect survivors (up to 6) | Partial: detects people in the image | scripts/yolo_person.py |
| 6 | Survivor grid box + marker on the map | **TODO** | needs survivor_tagger node |
| 7 | Mission status / progress / drone position display | Partial (RViz shows map, path, TF) | rviz/airmouse.rviz |
| 8 | Return to entry/exit point and exit | Sim (flies back to start, lands) | auto_explore.py (`return_home`) |
| 9 | 30 minute flight limit | **TODO** (mission timer) | |
| 10 | Emergency stop / abort | Partial: Ctrl+C lands; mode change hands control back | auto_explore.py |
| 11 | Failsafes: low battery, link loss, geofence | **TODO** (ArduPilot FS_* / FENCE_* params) | |
| 12 | No GPS / no internet | Sim OK. Real drone needs optical flow or lidar odometry | |
| 13 | Prop guards, <10 kg, 2x2 ft launch area | Hardware | |

## Arena in the brief vs. worlds/airmouse_maze.sdf
Your new world follows the brief: 15 x 15 m, 2.44 m (8 ft) walls, 4 x 4 rooms with ~1 m doorways, 1.2 m entrance,
2 ft launch pad outside the entrance. Net is visual only (not seen by the 2D lidar, which is fine).

## Known gaps for survivor search
- The explorer flies with a fixed heading, so the camera always looks the same way. To find survivors in rooms it needs
  (a) yaw control and (b) a "camera coverage" map so it visits/looks into every room, not only the lidar-mapped corridors.
- The SWEEP stage now makes the drone enter every room (tested: 16/16 rooms). It still does not turn to look; yaw control is next.
- The Iris model is ~0.7 m wide with props; doorways are ~1.0 m. Tight. A real drone for this arena should be smaller.
