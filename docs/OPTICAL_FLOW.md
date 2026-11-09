# Optical flow + IMU odometry (instead of Gazebo ground truth)

Chain:  IMU + optical flow + rangefinder -> ArduPilot EKF3 -> scripts/odom_from_ekf.py -> TF odom->base_link
        -> slam_toolbox (scan matching corrects the drift with the lidar) -> map

## 1. ArduPilot parameters (type in the MAVProxy window, then reboot SITL: `param set ...` then `reboot`)
    param set SIM_FLOW_ENABLE 1        # simulated flow sensor (SITL)
    param set FLOW_TYPE 10             # 10 = SITL
    param set RNGFND1_TYPE 100         # simulated downward rangefinder (SITL)  [if missing: check your params]
    param set EK3_SRC1_POSXY 0         # no GPS position
    param set EK3_SRC1_VELXY 5         # optical flow velocity
    param set EK3_SRC1_POSZ 1          # barometer height (or 2 = rangefinder)
    param set EK3_SRC1_VELZ 0
    param set EK3_SRC1_YAW 1           # compass (use 6 = external yaw if the compass is bad indoors)
    param set GPS1_TYPE 0              # GPS off
    param set ARMING_CHECK 0           # sim only
Names can differ by ArduPilot version - confirm with `param show EK3_SRC1*` and `param show FLOW*`.

## 2. Use it in the pipeline
1. Edit config/bridge.yaml: delete (or comment) the `/tf` entry so Gazebo's ground-truth odom->base_link is not published.
2. Start SITL with a second MAVLink output so this node does not share a port:
   add `--out=127.0.0.1:14552` to run/2_sitl.sh.
3. Run in a new terminal after mapping is up:  ./run/7_ekf_odom.sh
4. Check:  ros2 run tf2_ros tf2_echo odom base_link   (must show values that change when the drone moves)

## 3. slam.yaml (already scan-matching; keep these)
    do_loop_closing: true
    use_scan_matching: true
    minimum_travel_distance: 0.1
    minimum_travel_heading: 0.1

## Notes
- Flow needs a textured floor and a working rangefinder height (0.3-8 m); a plain grey floor gives no flow.
- Real hardware: same idea. Flow sensor (e.g. MTF-01 / PMW3901 + lidar) is wired to the flight controller; ArduPilot EKF3 does the fusing.
