"""All tunable numbers (ROS parameters override them). Edit DEFAULTS to change behaviour."""



DEFAULTS = dict(
    cruise_speed=0.5,        # m/s along the path (slow = cleaner 2D map)
    max_speed=0.6,           # m/s hard limit
    takeoff_alt=0.5,         # m. Lidar is ~0.3 m above the drone origin; boxes are 1 m tall
    cell_factor=4,           # planning cell = SLAM resolution x this (0.05 x 4 = 0.2 m)
    inflate_m=0.4,           # planner keeps path centre this far from walls (entrance is 1.2 m wide)
    pref_clear_m=0.8,        # planner prefers paths with this much clearance
    proximity_penalty=4.0,   # extra path cost when closer than pref_clear_m
    min_frontier_cells=1,    # frontier clusters smaller than this are ignored (1 = keep 1-cell doorways)
    min_unknown_cells=6,     # unknown patches smaller than this (cells of 0.2 m) are noise, not unexplored space
    lookahead_m=0.8,         # how far ahead on the path to aim
    goal_tol_m=0.6,          # frontier goal counts as reached inside this
    safe_radius_m=0.4,       # lidar safety bubble around the drone centre
    reaction_s=1.1,          # time budget for command latency (~0.3 s) + velocity lag (~0.4 s) + margin
    brake_decel=0.5,         # m/s^2 braking assumed by the stopping-distance barrier (conservative)
    accel_limit=0.8,         # m/s^2 limit on how fast the commanded velocity may change
    lookahead_min_m=0.4,     # shortest look-ahead (used when slow or close to walls)
    smooth_clear_m=0.45,     # smoothed path / look-ahead shortcuts must keep this clearance
    speck_cells=6,           # occupied specks smaller than this (5 cm cells, after joining gaps of 1 cell) are map noise
    return_retry_s=15.0,     # home looks blocked: keep re-planning this long before landing at the nearest safe point
    planner='astar',         # 'astar' = A* route to the chosen goal;  'dijkstra' = old behaviour (route read from the distance map)
    home_tol_m=0.25,         # RETURN ends (and the drone lands) this close to the launch point
    final_approach_max_m=1.5,  # if home itself can't be planned to: longest straight last leg allowed
    pref_land_clear_m=0.5,   # when home is unreachable, prefer a landing spot with this much clearance
    ret_stuck_max=3,         # this many 'no progress' events while returning -> land where it is
    replan_period_s=1.0,
    stuck_timeout_s=20.0,    # no 0.3 m progress for this long -> skip the goal
    finish_after_n_empty=4,  # consecutive plans without frontier -> exploration done
    blacklist_radius_m=1.0,
    return_home=True,
    entry_forward_m=2.0,     # after take-off fly this far straight ahead (through the entrance); 0 = skip
    gate_m=1.0,              # while exploring, never plan behind this line (measured forward from the start)
    keep_inside=True,        # enable the gate above
    enter_timeout_s=90.0,
    sweep_rooms=True,        # after mapping, visit every reachable area (all rooms)
    visit_radius_m=0.8,      # an area counts as visited when the drone has been this close
    min_sweep_cells=6,       # ignore unvisited specks smaller than this (cells of 0.2 m)
    room_erode_m=0.6,        # space further than this from any wall = 'room core'; doorways are narrower
    room_erode2_m=0.8,       # second, larger size for rooms with wide (1.2-1.6 m) doors; 0 = off
    room_min_cells=4,        # smallest room core to care about (cells)
    room_visit_m=0.3,        # drone counts as inside a room when this close to its core
    sweep_goal_tol_m=0.3,    # how close to a room's centre the drone must get
    cover_sweep=False,       # after the rooms: also visit big areas the drone has not been near (adds ~40% time)
    cover_radius_m=1.5,      # an area counts as covered when the drone has been this close to it
    cover_min_cells=8,       # ignore uncovered pockets smaller than this (cells of 0.2 m, inflated space)
    cover_goal_tol_m=0.6,    # a coverage goal counts as reached this close
    # ---- safety nets (the mission can never hang forever) ----
    explore_limit_s=1200.0,  # mission time after which exploring/sweeping stops and the drone returns
    return_limit_s=150.0,    # if the way home is not finished within this, land where it is
    mission_limit_s=1680.0,  # hard stop (28 min): land wherever the drone is
    data_timeout_s=10.0,     # pose / scan / map missing for this long -> land
    land_retry_s=2.0,        # LAND mode is re-sent this often until the drone is disarmed
    alt_min_m=0.15,          # flying lower than this (relative to take-off) for alt_bad_s -> land
    alt_max_m=1.2,           # flying higher than this for alt_bad_s -> land (ceiling is 2.4 m)
    alt_bad_s=3.0,
    heartbeat_timeout_s=5.0, # no MAVLink heartbeat for this long (simulation seconds) -> hold, then land
    arm_give_up_s=60.0,      # arming keeps failing for this long -> give up and say why
    map_stale_warn_s=30.0,   # /map not updated for this long while its publisher is alive -> warn
    look_around=True,        # stop and turn one full circle now and then so the camera can see people
    look_rate=0.6,           # turn speed (rad/s): one circle takes about 10.5 s
    look_every_m=5.0,        # ... after this many metres of flight, and after each newly visited room
    look_min_gap_s=20.0,     # ... but never more often than this
)
