#!/usr/bin/env python3
"""Simulation-only mapper: builds /map straight from the lidar + Gazebo's exact pose (no SLAM).

Why: with exact poses, a log-odds occupancy grid gives crisp walls. If THIS map is clean but the
slam_toolbox map is not, the problem is SLAM tuning / timing, not the sensor. On real hardware use SLAM.
Publishes /map (OccupancyGrid, latched) and a static TF map->odom (identity).
Needs TF odom->base_link->laser (from the bridge) and /scan.
"""
import math
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
except ImportError:
    rclpy = None

L_FREE, L_OCC, L_MIN, L_MAX = -0.4, 0.85, -4.0, 4.0


def integrate(lo, ox, oy, res, x, y, yaw_laser, ranges, a0, da, rmin, rmax, max_free):
    """Update log-odds grid `lo` (H x W) with one scan taken from laser pose (x, y, yaw_laser)."""
    H, W = lo.shape
    r = np.asarray(ranges, float)
    ang = yaw_laser + a0 + np.arange(len(r)) * da
    hit = np.isfinite(r) & (r >= rmin) & (r < rmax - 0.05)
    reach = np.where(hit, r, np.minimum(max_free, rmax))
    reach = np.where(np.isnan(reach), 0.0, reach)
    step = res * 0.5
    n = int(np.ceil(rmax / step))
    ts = np.arange(1, n + 1) * step
    mask = ts[None, :] < (reach[:, None] - res * 0.5)           # free cells: before the hit cell
    X = x + np.cos(ang)[:, None] * ts[None, :]
    Y = y + np.sin(ang)[:, None] * ts[None, :]
    J = np.floor((X - ox) / res).astype(int); I = np.floor((Y - oy) / res).astype(int)
    ok = mask & (I >= 0) & (I < H) & (J >= 0) & (J < W)
    free = np.unique(I[ok] * W + J[ok])
    lo.flat[free] = np.maximum(lo.flat[free] + L_FREE, L_MIN)
    hx = x + np.cos(ang[hit]) * r[hit]; hy = y + np.sin(ang[hit]) * r[hit]
    hj = np.floor((hx - ox) / res).astype(int); hi = np.floor((hy - oy) / res).astype(int)
    k = (hi >= 0) & (hi < H) & (hj >= 0) & (hj < W)
    occ = np.unique(hi[k] * W + hj[k])
    lo.flat[occ] = np.minimum(lo.flat[occ] + L_OCC - L_FREE, L_MAX)   # cancel the free update on hit cells
    return lo


def to_occupancy(lo):
    m = np.full(lo.shape, -1, np.int8)
    m[lo < -0.2] = 0
    m[lo > 0.6] = 100
    return m


if rclpy is not None:
    class GridMapper(Node):
        def __init__(self):
            super().__init__('grid_mapper')
            for k, v in (('resolution', 0.05), ('size_m', 40.0), ('max_free_m', 6.0), ('publish_hz', 1.0)):
                self.declare_parameter(k, v)
            self.res = self.get_parameter('resolution').value
            size = self.get_parameter('size_m').value
            self.max_free = self.get_parameter('max_free_m').value
            self.W = self.H = int(size / self.res)
            self.ox = self.oy = -size / 2.0
            self.lo = np.zeros((self.H, self.W))
            self.lo[:] = 0.0
            self.seen = np.zeros((self.H, self.W), bool)
            self.tfbuf = Buffer(); self.tfl = TransformListener(self.tfbuf, self)
            st = StaticTransformBroadcaster(self)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = 'map', 'odom'
            t.transform.rotation.w = 1.0
            st.sendTransform(t); self._st = st
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            self.pub = self.create_publisher(OccupancyGrid, '/map', qos)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            self.create_timer(1.0 / self.get_parameter('publish_hz').value, self.publish)
            self.used = self.dropped = 0
            self.create_timer(10.0, lambda: self.get_logger().info(
                'scans used %d, dropped %d (no TF)' % (self.used, self.dropped)))
            self.get_logger().info('grid_mapper running (no SLAM): map from /scan + TF')

        def on_scan(self, s):
            frame = s.header.frame_id or 'laser'
            tf = None
            for stamp, tmo in ((s.header.stamp, 0.05), (rclpy.time.Time(), 0.0)):   # exact time, else latest
                try:
                    tf = self.tfbuf.lookup_transform('map', frame, stamp, rclpy.duration.Duration(seconds=tmo))
                    break
                except Exception:
                    continue
            if tf is None:
                self.dropped += 1
                return
            self.used += 1
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            before = self.lo != 0
            integrate(self.lo, self.ox, self.oy, self.res, tf.transform.translation.x,
                      tf.transform.translation.y, yaw, s.ranges, s.angle_min, s.angle_increment,
                      s.range_min, s.range_max, self.max_free)
            self.seen |= (self.lo != 0)

        def publish(self):
            m = OccupancyGrid()
            m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = 'map'
            m.info.resolution = self.res; m.info.width = self.W; m.info.height = self.H
            m.info.origin.position.x = self.ox; m.info.origin.position.y = self.oy
            m.info.origin.orientation.w = 1.0
            occ = to_occupancy(self.lo)
            occ[~self.seen] = -1
            m.data = occ.flatten().tolist()
            self.pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(GridMapper())


if __name__ == '__main__':
    main()
