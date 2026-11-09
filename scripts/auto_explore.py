#!/usr/bin/env python3
"""
auto_explore.py - start the autonomous explorer.   python3 auto_explore.py --ros-args -p cruise_speed:=0.3

The code lives in the  airmouse/  folder, one small file per job (see docs/CODE_MAP.md):
  params.py    all tunable numbers          logic.py     mission state machine
  datatypes.py Grid / Scan containers       pathplan.py  Dijkstra, frontiers, rooms
  mapgrid.py   map down-sample / inflate    safety.py    stopping filter, smoothing
  lidar.py     scan clean-up                node.py      ROS + MAVLink + take-off/landing
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from airmouse import datatypes, deps, lidar, logic, mapgrid, node, params, pathplan, safety   # noqa: E402,F401
main = node.main

# Backward compatibility: test scripts do `import auto_explore as ae; ae.Logic, ae.DEFAULTS, ae.prepare ...`
for _m in (params, datatypes, mapgrid, pathplan, lidar, safety, logic, node):
    globals().update({k: v for k, v in vars(_m).items() if not k.startswith('__')})

if __name__ == '__main__':
    main()
