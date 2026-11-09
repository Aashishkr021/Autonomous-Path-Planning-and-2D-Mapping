"""ROS 2 node: TF / topics / MAVLink I/O, take-off, landing, health checks, RViz markers."""
import math
import os
import sys
import time

import numpy as np
from .datatypes import Grid, Scan
from .lidar import scan_to_base
from .logic import Logic
from .mapgrid import save_map
from .params import DEFAULTS
from .safety import alt_speed_factor

try:  # lets the planner be tested without ROS installed
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Path
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float32
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import Marker
except ImportError:
    rclpy = None
    Node = object


class ExploreNode(Node):
    def __init__(self):
        super().__init__('auto_explore', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.declare_parameter('mavlink', 'udp:127.0.0.1:14551')
        self.declare_parameter('map_prefix', os.path.expanduser('~/nidar_slam/airmouse_map'))
        self.declare_parameter('land_on_exit', True)
        self.P = {k: self.get_parameter(k).value for k in DEFAULTS}
        self.logic = Logic(**self.P)
        self.map_prefix = self.get_parameter('map_prefix').value
        self.land_on_exit = self.get_parameter('land_on_exit').value

        self.scan = None
        self.grid = None
        self.z0 = None
        self.phase = 'WAIT'
        self.done = False
        self.armed = False
        self.mode = ''
        self.t_try = -1e9
        self.t_takeoff = 0.0
        self.t_land = 0.0
        self.t_log = -1e9
        self.shown_plan = -1
        self.t0_sim = None          # mission clock: starts at take-off command, stops when landed
        self.t0_wall = None
        self.stage_t = {}           # first mission time each logic state was seen
        self.last_state = ''
        self.time_pub = self.create_publisher(Float32, '/explore/mission_time_s', 1)
        self.t_scan = None          # when the last scan arrived (sim time)
        self.t_map = None           # when the last /map arrived (sim time)
        self.t_ok = None            # last time pose + scan + map were all available
        self.t_hb = None            # sim time of the last autopilot HEARTBEAT
        self.t_arm0 = None          # when arming was first tried
        self.prearm = []            # last PreArm / arming messages from the autopilot
        self.t_alt_bad = None
        self.look, self.look_last_xy, self.look_last_t, self.look_rooms = None, None, 0.0, 0
        self.t_pubchk = -1e9
        self.map_pub_alive = True
        self.laser_off = (0.0, 0.0, 0.0)   # base_link -> laser (x, y, yaw), read once from TF
        self.t_stopped_log = -1e9
        self.errors = 0
        self.land_cmds = 0

        self.tfbuf = Buffer()
        self.tfl = TransformListener(self.tfbuf, self)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.path_pub = self.create_publisher(Path, '/explore/path', 1)
        self.goal_pub = self.create_publisher(Marker, '/explore/goal', 1)

        from pymavlink import mavutil
        self.mavutil = mavutil
        self.m = mavutil.mavlink_connection(self.get_parameter('mavlink').value)
        self.get_logger().info('Waiting for MAVLink heartbeat. In the MAVProxy window run: '
                               'output add 127.0.0.1:14551  (or start sim_vehicle.py with '
                               '--out=127.0.0.1:14551)')
        while rclpy.ok() and self.m.wait_heartbeat(timeout=3) is None:
            self.get_logger().warn('still waiting for heartbeat...')
        self.get_logger().info('MAVLink connected.')
        self.create_timer(0.1, self.loop)

    # ---- helpers ----------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_scan(self, msg):
        r, a = scan_to_base(msg.ranges, msg.angle_min, msg.angle_increment, *self.laser_off)
        self.scan = Scan(r, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max, a)
        self.t_scan = self.now()

    def _on_map(self, msg):
        w, h = msg.info.width, msg.info.height
        if w * h == 0 or len(msg.data) != w * h or msg.info.resolution <= 0:
            return                                    # malformed map: keep the previous one
        data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        q = msg.info.origin.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.grid = Grid(data, msg.info.resolution, msg.info.origin.position.x,
                         msg.info.origin.position.y, yaw)
        self.t_map = self.now()

    def tf_pose(self, parent):
        try:
            t = self.tfbuf.lookup_transform(parent, 'base_link', rclpy.time.Time())
        except Exception:
            return None
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        if stamp > 0 and self.now() - stamp > 1.0:     # TF publisher stopped: do not fly on an old pose
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        tr = t.transform.translation
        if not all(math.isfinite(v) for v in (tr.x, tr.y, tr.z, yaw)):
            return None
        return tr.x, tr.y, tr.z, yaw

    def rel_alt(self):
        o = self.tf_pose('odom')
        return None if (o is None or self.z0 is None) else o[2] - self.z0

    def pump(self):
        """Read all pending MAVLink messages; track armed state and flight mode."""
        mav = self.mavutil.mavlink
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                break
            kind = msg.get_type()
            if kind == 'HEARTBEAT' and msg.type != mav.MAV_TYPE_GCS \
                    and msg.get_srcSystem() == self.m.target_system:
                self.armed = bool(msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = self.mavutil.mode_string_v10(msg)
                self.t_hb = self.now()          # SIM time: SITL's heartbeat follows the simulator clock, so a slow Gazebo must not look like a dead link
            elif kind == 'STATUSTEXT':
                self.get_logger().info('[autopilot] %s' % msg.text)
                if 'rearm' in msg.text or 'rm:' in msg.text or 'rming' in msg.text:
                    self.prearm = (self.prearm + [msg.text])[-3:]

    def send_vel(self, vx, vy_left, vz_down, yaw_rate_ccw=0.0):
        m = self.m
        # BODY_NED: x forward, y RIGHT, z down. Mask 1479 = velocity + yaw rate (0 = keep heading).
        # NED yaw rate is clockwise-positive, so a counter-clockwise turn is sent negated.
        m.mav.set_position_target_local_ned_send(
            0, m.target_system, m.target_component,
            self.mavutil.mavlink.MAV_FRAME_BODY_NED, 1479,
            0, 0, 0, vx, -vy_left, vz_down, 0, 0, 0, 0, -yaw_rate_ccw)

    def look_around(self, t, pose, cmd, vz):
        """Hold position and turn one full circle so the forward camera sees all around.
        Starts every look_every_m metres of flight and after each newly visited room. True while turning."""
        P = self.P
        if not P['look_around']:
            return False
        x, y, yaw = pose[0], pose[1], pose[3]
        lk = self.look
        if lk is None:
            if cmd.state not in ('EXPLORE', 'SWEEP') or self.phase != 'EXPLORE':
                return False
            if self.look_last_xy is None:
                self.look_last_xy, self.look_last_t = (x, y), t
            n_vis = sum(1 for r_ in self.logic.room_reg if r_['visited'])
            moved = math.hypot(x - self.look_last_xy[0], y - self.look_last_xy[1])
            new_room = n_vis > self.look_rooms
            if (moved >= P['look_every_m'] or new_room) and t - self.look_last_t >= P['look_min_gap_s']:
                self.look = lk = dict(t0=t, yaw_prev=yaw, turned=0.0)
                self.look_rooms = n_vis
                self.log('Look-around: turning one full circle (camera scan)', 1.0)
            else:
                return False
        d = (yaw - lk['yaw_prev'] + math.pi) % (2 * math.pi) - math.pi
        lk['turned'] += abs(d)
        lk['yaw_prev'] = yaw
        timeout = 2.0 * (2 * math.pi / P['look_rate']) + 3.0
        if lk['turned'] >= 2 * math.pi - 0.15 or t - lk['t0'] > timeout:
            if lk['turned'] < math.pi:                       # the autopilot is not turning: stop trying
                self.get_logger().warning('Look-around: yaw did not turn (%.1f rad) - disabled' % lk['turned'])
                P['look_around'] = False
            self.look, self.look_last_xy, self.look_last_t = None, (x, y), t
            self.logic.ref_t = t                             # the stuck timer must not count the turn
            return False
        self.send_vel(0.0, 0.0, vz, P['look_rate'])
        self.publish_viz()
        return True

    def send_takeoff(self):
        m = self.m
        m.mav.command_long_send(m.target_system, m.target_component,
                                self.mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                0, 0, 0, 0, 0, 0, 0, float(self.P['takeoff_alt']))

    def arm(self):
        m = self.m
        m.set_mode('GUIDED')
        m.mav.command_long_send(m.target_system, m.target_component,
                                self.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 1, 0, 0, 0, 0, 0, 0)

    def log(self, msg, period=5.0):
        t = self.now()
        if t - self.t_log >= period:
            self.t_log = t
            self.get_logger().info(msg)

    # ---- main loop (10 Hz, sim time) ---------------------------------------
    def loop(self):
        self.pump()
        t = self.now()
        ph = self.phase
        hb_age = None if self.t_hb is None else self.now() - self.t_hb
        if ph in ('TAKEOFF', 'EXPLORE') and not self.armed and hb_age is not None and hb_age < 2.0:
            self.get_logger().error('Drone disarmed in flight (crash / failsafe / pilot). Stopping the explorer.')
            self.report('disarmed in flight')
            self.finish()
            return

        if ph == 'WAIT':
            missing = []
            if self.scan is None:
                missing.append('/scan')
            if self.grid is None:
                missing.append('/map (is slam_toolbox running and has the drone moved/scanned?)')
            for a, b in (('odom', 'base_link'), ('base_link', 'laser'), ('map', 'odom')):
                if not self.tfbuf.can_transform(a, b, rclpy.time.Time()):
                    missing.append('TF %s -> %s' % (a, b))
            if missing:
                self.log('Waiting for: ' + ', '.join(missing))
                return
            self.z0 = self.tf_pose('odom')[2]
            try:                                           # where the lidar sits relative to base_link
                lt = self.tfbuf.lookup_transform('base_link', 'laser', rclpy.time.Time()).transform
                lyaw = math.atan2(2 * (lt.rotation.w * lt.rotation.z + lt.rotation.x * lt.rotation.y),
                                  1 - 2 * (lt.rotation.y ** 2 + lt.rotation.z ** 2))
                self.laser_off = (lt.translation.x, lt.translation.y, lyaw)
                self.get_logger().info('Lidar mount: x=%.2f y=%.2f yaw=%.2f rad' % self.laser_off)
            except Exception:
                pass
            self.get_logger().info('All inputs present. Arming.')
            self.phase = 'ARM'

        elif ph == 'ARM':
            if self.armed and self.mode == 'GUIDED':
                self.send_takeoff()
                self.t_takeoff = t
                self.t0_sim = t
                self.t0_wall = time.time()
                self.phase = 'TAKEOFF'
                self.get_logger().info('Armed. Taking off to %.2f m' % self.P['takeoff_alt'])
            elif t - self.t_try > 3.0:
                self.t_try = t
                if self.t_arm0 is None:
                    self.t_arm0 = t
                elif t - self.t_arm0 > self.P['arm_give_up_s']:
                    self.get_logger().error(
                        'ARMING FAILED for %.0f s - giving up. Last autopilot messages: %s. '
                        'Typical causes: EKF not ready (wait 1-2 min after SITL start), compass/GPS pre-arm '
                        'checks (set ARMING_CHECK 0 in SITL), or another program holding GUIDED.'
                        % (self.P['arm_give_up_s'], ' | '.join(self.prearm) or 'none'))
                    self.phase, self.done = 'DONE', True
                    return
                self.arm()   # retries until the EKF is ready; reasons appear as [autopilot] lines

        elif ph == 'TAKEOFF':
            alt = self.rel_alt()
            if self.mode != 'GUIDED':
                self.get_logger().warn('Mode left GUIDED - stopping. You have control.')
                self.phase = 'STOPPED'
            elif alt is not None and alt >= 0.85 * self.P['takeoff_alt']:
                self.get_logger().info('Airborne. Exploring.')
                self.phase = 'EXPLORE'
            elif t - self.t_takeoff > 10.0:
                self.t_takeoff = t
                self.send_takeoff()
                self.n_takeoff = getattr(self, 'n_takeoff', 0) + 1
                if self.n_takeoff >= 6:
                    self.start_landing('take-off did not reach the altitude in 60 s')

        elif ph == 'EXPLORE':
            if self.mode != 'GUIDED':
                self.get_logger().warn('Mode left GUIDED - stopping. You have control.')
                self.phase = 'STOPPED'
                return
            pose = self.tf_pose('map')
            alt = self.rel_alt()
            if alt is not None and not math.isfinite(alt):
                alt = None
            scan_age = 1e9 if self.t_scan is None else t - self.t_scan
            if t - self.t_pubchk > 2.0:                                  # missing SLAM vs. merely unchanged map
                self.t_pubchk = t
                self.map_pub_alive = self.count_publishers('/map') > 0
            map_age = None if self.t_map is None else t - self.t_map
            if map_age is not None and map_age > self.P['map_stale_warn_s'] and self.map_pub_alive:
                self.log('WARNING: /map not updated for %.0f s (SLAM alive but busy, or the drone is not moving)'
                         % map_age, 15.0)
            hb_bad = hb_age is not None and hb_age > self.P['heartbeat_timeout_s']
            bad = [n for n, b in (('pose (TF map->base_link)', pose is None),
                                  ('altitude', alt is None),
                                  ('/scan (age %.1fs)' % min(scan_age, 999), scan_age > 2.0 or self.scan is None),
                                  ('/map' if self.map_pub_alive else '/map (slam_toolbox is NOT running)',
                                   self.grid is None or not self.map_pub_alive),
                                  ('MAVLink heartbeat (%.1fs)' % (hb_age or 0), hb_bad)) if b]
            if bad:
                self.send_vel(0.0, 0.0, 0.0)                         # hold still while data is unreliable
                if self.t_ok is None:
                    self.t_ok = t
                self.log('HOLD: unreliable / missing: %s' % ', '.join(bad), 3.0)
                if t - self.t_ok > self.P['data_timeout_s']:
                    self.start_landing('%s missing for %.0f s' % (', '.join(bad), self.P['data_timeout_s']))
                return
            # altitude limits: outside [alt_min, alt_max] for alt_bad_s -> land
            if alt < self.P['alt_min_m'] or alt > self.P['alt_max_m']:
                self.t_alt_bad = t if self.t_alt_bad is None else self.t_alt_bad
                if t - self.t_alt_bad > self.P['alt_bad_s']:
                    self.start_landing('altitude %.2f m is outside the allowed %.2f-%.2f m'
                                       % (alt, self.P['alt_min_m'], self.P['alt_max_m']))
                    return
            else:
                self.t_alt_bad = None
            self.t_ok = None
            try:
                cmd = self.logic.update(t, (pose[0], pose[1], pose[3]), self.scan, self.grid)
                self.errors = 0
            except Exception as e:                                    # a planner bug must never leave the drone hovering
                import traceback
                self.errors += 1
                self.get_logger().error('planner error (%d): %s\n%s' % (self.errors, e, traceback.format_exc()))
                self.send_vel(0.0, 0.0, 0.0)
                if self.errors >= 20:
                    self.start_landing('planner kept failing')
                return
            vz = -float(np.clip(1.0 * (self.P['takeoff_alt'] - alt), -0.3, 0.3))  # hold altitude
            if abs(self.P['takeoff_alt'] - alt) < 0.15 and self.look_around(t, pose, cmd, vz):
                return
            fac = alt_speed_factor(self.P['takeoff_alt'] - alt)       # fix the height first, then move
            self.send_vel(cmd.vx * fac, cmd.vy * fac, vz)
            self.publish_viz()
            self.time_pub.publish(Float32(data=float(self.elapsed())))
            if cmd.state != self.last_state:
                self.last_state = cmd.state
                self.stage_t[cmd.state] = t - self.t0_sim
                self.get_logger().info('Stage -> %s%s' % (cmd.state, ('  (' + self.logic.reason + ')') if self.logic.reason else ''))
            self.log('[%s] %s  pos=(%.1f, %.1f)  frontiers=%d  goal=%s  v=(%.2f, %.2f)' % (
                self.clock_str(), cmd.state, pose[0], pose[1], self.logic.n_frontiers,
                None if self.logic.goal is None else '(%.1f, %.1f)' % self.logic.goal,
                cmd.vx, cmd.vy))
            if cmd.state == 'DONE':
                self.start_landing('mission finished' + ((' - ' + self.logic.reason) if self.logic.reason else ''))

        elif ph == 'STOPPED':
            if not self.armed:                    # whoever has control has landed: just save and exit
                self.report('stopped (disarmed)')
                self.finish()
            else:
                self.log('Manual control: flight mode is %s. The explorer is not sending commands. '
                         'Type  mode land  in MAVProxy to land, or Ctrl+C here to land and save.' % self.mode, 10.0)

        elif ph == 'LAND':
            self.land_step(t)

    def start_landing(self, why):
        self.get_logger().warn('Landing: %s' % why)
        self.send_vel(0.0, 0.0, 0.0)
        self.m.set_mode('LAND')
        self.t_land = self.now()
        self.t_low = None
        self.land_cmds = 1
        self.phase = 'LAND'

    def land_step(self, t):
        """LAND phase: keep asking for LAND mode until the drone is down and disarmed."""
        alt = self.rel_alt()
        if self.armed and self.mode != 'LAND' and t - self.t_land > self.P['land_retry_s'] * self.land_cmds:
            self.m.set_mode('LAND')                       # re-send: a single UDP packet can get lost
            self.land_cmds += 1
            if self.land_cmds % 5 == 0:
                self.get_logger().warn('LAND not accepted yet (mode=%s). Retrying. '
                                       'You can also type  mode land  in the MAVProxy window.' % self.mode)
        low = alt is not None and alt < 0.06
        self.t_low = (self.t_low if self.t_low is not None else t) if low else None
        if (not self.armed) or (self.t_low is not None and t - self.t_low > 3.0):
            self.report('landed')
            self.finish()
        elif t - self.t_land > 120:
            self.get_logger().error('Drone did not confirm landing after 120 s. Type  mode land  in MAVProxy.')
            self.report('landing not confirmed')
            self.finish()

    def elapsed(self):
        return 0.0 if self.t0_sim is None else self.now() - self.t0_sim

    def clock_str(self):
        e = self.elapsed()
        return '%02d:%02d' % (e // 60, e % 60)

    def report(self, why):
        if self.t0_sim is None:
            return
        sim = self.now() - self.t0_sim
        wall = time.time() - self.t0_wall
        lines = ['MISSION TIME (take-off -> %s): %d min %02d s simulated  (%d min %02d s real time)' % (
                 why, sim // 60, sim % 60, wall // 60, wall % 60)]
        names = list(self.stage_t.items())
        for i, (k, v) in enumerate(names):
            end = names[i + 1][1] if i + 1 < len(names) else sim
            lines.append('   %-8s %3d s' % (k, end - v))
        if sim > 1800:
            lines.append('   WARNING: over the 30 min mission limit')
        txt = '\n'.join(lines)
        self.get_logger().info('\n' + txt)
        try:
            with open(self.map_prefix + '_mission.txt', 'w') as f:
                f.write(txt + '\n')
        except Exception:
            pass
        self.t0_sim = None          # report once

    def publish_viz(self):
        lg = self.logic
        if lg.plan_id == self.shown_plan:
            return
        self.shown_plan = lg.plan_id
        path = Path()
        path.header.frame_id = 'map'
        for x, y in lg.path:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)
        mk = Marker()
        mk.header.frame_id = 'map'
        mk.ns, mk.id, mk.type = 'goal', 0, Marker.SPHERE
        mk.scale.x = mk.scale.y = mk.scale.z = 0.3
        mk.color.r, mk.color.g, mk.color.a = 1.0, 0.8, 1.0
        if lg.goal:
            mk.action = Marker.ADD
            mk.pose.position.x, mk.pose.position.y = lg.goal
            mk.pose.orientation.w = 1.0
        else:
            mk.action = Marker.DELETE
        self.goal_pub.publish(mk)

    def finish(self):
        self.save()
        self.phase = 'DONE'
        self.done = True

    def save(self):
        if self.grid is not None:
            save_map(self.grid, self.map_prefix)
            self.get_logger().info('Map saved: %s.pgm / .yaml' % self.map_prefix)

    def shutdown(self):
        """Called on Ctrl+C / exit."""
        try:
            self.report('stopped by user')
            if self.phase in ('TAKEOFF', 'EXPLORE', 'LAND', 'STOPPED') and self.armed and self.land_on_exit:
                self.get_logger().info('Stopping: landing.')
                for _ in range(3):                     # repeat: one UDP packet can get lost
                    self.m.set_mode('LAND')
                    time.sleep(0.1)
            if self.phase != 'DONE':
                self.save()
        except Exception as e:  # never crash while shutting down
            self.get_logger().warn('shutdown: %s' % e)


def main():
    if rclpy is None:
        print('ROS 2 python packages not found. Run: source /opt/ros/jazzy/setup.bash')
        sys.exit(1)
    rclpy.init(args=sys.argv)
    import scipy
    print('auto_explore: numpy %s, scipy %s, python %s' % (np.__version__, scipy.__version__, sys.version.split()[0]))
    node = ExploreNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except Exception:                                  # any crash: still land and save (the drone must never be left hovering)
        import traceback
        traceback.print_exc()
    node.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
