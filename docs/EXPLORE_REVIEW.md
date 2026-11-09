# auto_explore.py - review items and what was done

Tested offline (planner harness on your 15x15 building and on the earlier 4x4-room maze, command latency
0.3-0.5 s, drone lag 0.4 s). NOT yet flown in Gazebo - the first Gazebo run is the real test.

| Item | Verdict | What changed |
|---|---|---|
| A unknown cells traversable | real bug | start-cell exception only opens KNOWN-free neighbours |
| B to_cell int() | real bug | floor(), bounds helper `in_bounds`, map-edge safe slicing |
| C straight-line home leg | real risk | full inflated-grid path to home; a straight final leg is used only if it is clear and <= `final_approach_max_m`; otherwise lands at the best reachable spot; `return_limit_s` and `ret_stuck_max` stop endless returns |
| D safety filter | improved | stopping-distance barrier (`reaction_s`, `brake_decel`), `accel_limit`, result always satisfies constraints, REP-117 handling of inf / -inf / NaN (-inf = obstacle, not free space), lidar mount offset read from TF |
| E unvisited_targets | tested, kept OFF | `cover_sweep` exists but adds about 40% time for little gain; rooms are covered by the room sweep |
| F room detection | improved | two erosion sizes (narrow + wide doors), corridor = component at the entry (not "largest"), same-room de-duplication, persistent room list |
| G path cutting | real risk | look-ahead is collision-checked on the inflated map and shortened when blocked |
| H health monitoring | added | scan / map / pose freshness, SLAM missing vs map unchanged, MAVLink heartbeat loss, arming failure with the autopilot's reason, take-off failure, disarm in flight, mode change |
| I map origin yaw | handled | origin orientation used in to_cell / to_world; malformed maps ignored |
| J altitude | improved | limits (`alt_min_m`, `alt_max_m`), invalid altitude = hold then land, horizontal speed is scaled to zero while height error > 0.25 m |
| K speed | done | scipy csgraph Dijkstra, one multi-source search, 26 ms for a 30x30 m map |
| L smoothing | done | line-of-sight smoothing with the same clearance, re-sampled every 0.2 m |
| M NumPy/SciPy | done | clear fix message at start-up, versions printed |

Run:  `./run/4_explore.sh`  (no extra -p flags needed for the 15x15 building).
Narrow 1 m corridors: `-p safe_radius_m:=0.35`.
