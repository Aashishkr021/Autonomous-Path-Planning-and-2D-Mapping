#!/usr/bin/env python3
"""Manual keyboard flight (testing only - do NOT run together with auto_explore.py)."""
import sys, time, termios, tty, select
from pymavlink import mavutil

CONN = 'udp:127.0.0.1:14551'
SPEED, VSPEED, YAW_RATE, TAKEOFF_ALT, KEY_TIMEOUT = 0.4, 0.2, 0.4, 0.4, 0.3

m = mavutil.mavlink_connection(CONN)
print("Waiting for heartbeat...")
m.wait_heartbeat()
print("Connected.")
HELP = """
 w/s : forward/back     a/d : left/right
 q/e : turn left/right  r/f : up/down
 space : stop           t : guided+arm+takeoff
 l : land               x : quit
"""

def send_vel(vx, vy, vz, yaw_rate):
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, 1479,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, yaw_rate)

def takeoff():
    m.set_mode('GUIDED'); time.sleep(1)
    m.arducopter_arm(); m.motors_armed_wait()
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, TAKEOFF_ALT)
    print("Taking off...")

def get_key():
    return sys.stdin.read(1) if select.select([sys.stdin], [], [], 0.05)[0] else None

old = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())
print(HELP)
vx = vy = vz = yr = 0.0
last = 0.0
try:
    while True:
        k = get_key()
        if k:
            last = time.time()
            if   k == 'w': vx, vy, vz, yr = SPEED, 0, 0, 0
            elif k == 's': vx, vy, vz, yr = -SPEED, 0, 0, 0
            elif k == 'a': vx, vy, vz, yr = 0, -SPEED, 0, 0
            elif k == 'd': vx, vy, vz, yr = 0, SPEED, 0, 0
            elif k == 'r': vx, vy, vz, yr = 0, 0, -VSPEED, 0
            elif k == 'f': vx, vy, vz, yr = 0, 0, VSPEED, 0
            elif k == 'q': vx, vy, vz, yr = 0, 0, 0, -YAW_RATE
            elif k == 'e': vx, vy, vz, yr = 0, 0, 0, YAW_RATE
            elif k == ' ': vx = vy = vz = yr = 0.0
            elif k == 't': takeoff()
            elif k == 'l': m.set_mode('LAND'); print("Landing...")
            elif k == 'x': break
        if time.time() - last > KEY_TIMEOUT:
            vx = vy = vz = yr = 0.0
        send_vel(vx, vy, vz, yr)
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
    print("\nBye.")
