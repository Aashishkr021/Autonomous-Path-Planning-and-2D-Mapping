# Code map - which file to open when something goes wrong

`scripts/auto_explore.py` only starts things. The real code is in `scripts/airmouse/`, one job per file:

| File | Lines | Job | Open it when... |
|---|---|---|---|
| `params.py` | ~60 | every tunable number (`DEFAULTS`) | you want to change speed, clearance, time limits |
| `datatypes.py` | ~20 | `Grid` (map) and `Scan` (lidar) containers | rarely |
| `mapgrid.py` | ~100 | shrink + inflate the SLAM map, cell<->metre, save map | drone thinks free space is a wall (or the reverse) |
| `pathplan.py` | ~300 | A* route to the goal, Dijkstra distance map, frontier choice, room detection, room tour | it picks a strange goal, misses a room, takes a long route |
| `lidar.py` | ~35 | clean a scan, move it to the drone centre | lidar mount / wrong-angle problems |
| `safety.py` | ~80 | stopping-distance filter, smoothing, altitude speed factor | it brushes walls, or is too timid |
| `logic.py` | ~350 | state machine ENTER > EXPLORE > SWEEP > RETURN > HOME > DONE | it stops early, does not go home, wrong stage order |
| `node.py` | ~470 | ROS topics, TF, MAVLink, take-off, LANDING, health checks | no take-off, no landing, "HOLD", arming errors |

Data flow: `node.py` reads pose/scan/map -> `logic.py` (uses `pathplan`, `mapgrid`, `safety`, `lidar`) returns a velocity -> `node.py` sends it.
Rule of thumb: **everything except `node.py` runs without ROS**, so you can test it on any computer.

## Test one part at a time (no Gazebo needed)
```bash
cd ~/nidar_airmouse/tests
python3 test_node.py       # health logic: pose lost, heartbeat lost, SLAM missing, altitude, disarm, arming failure
python3 test_planner.py    # whole mission on 2 simulated buildings x 3 seeds x 2 latencies (about 2-4 min)
```
Expected: `test_planner.py` prints `done: True`, `rooms_entered: 4` (new building) / `rooms: 16` (old maze), `end` within 0.2 m of (0, -7.87).

Change a parameter for one test run without editing files: in Python, `ae.DEFAULTS['cruise_speed'] = 0.3` before the run.

## Planner: A* vs Dijkstra
`planner='astar'` (default in `params.py`): Dijkstra still runs once per replan to learn the cost to every cell (needed to CHOOSE the nearest frontier / room / reachable home); then A* draws the actual route to that chosen goal. Set `planner='dijkstra'` to get the old behaviour back.
