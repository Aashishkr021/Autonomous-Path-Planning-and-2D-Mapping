#!/usr/bin/env python3

import csv
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from airmouse.survivors import (SURVIVOR_DEFAULTS, SurvivorTracker, base_to_map, box_to_base, grid_label,
                                refine_with_lidar)


class SurvivorTagger(Node):
    def __init__(self):
        super().__init__('survivor_tagger')
        for k, v in SURVIVOR_DEFAULTS.items():
            self.declare_parameter(k, v)
        self.declare_parameter('out_prefix', os.path.expanduser('~/nidar_airmouse/maps/survivors'))
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.p = {k: self.get_parameter(k).value for k in SURVIVOR_DEFAULTS}
        self.out = self.get_parameter('out_prefix').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tracker = SurvivorTracker(self.p)
        self.scan = None
        self.n_seen = 0
        self.last_n = 0
        self.tfbuf = Buffer()
        self.tfl = TransformListener(self.tfbuf, self)
        self.create_subscription(String, '/person_boxes', self.on_boxes, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(MarkerArray, '/survivors', latched)
        self.create_timer(1.0, self.publish_markers)
        os.makedirs(os.path.dirname(self.out), exist_ok=True)
        self.get_logger().info('survivor_tagger ready: waiting for /person_boxes, /scan and TF %s->%s'
                               % (self.map_frame, self.base_frame))
        self.save([])        # create the (empty) files straight away, so you can see the tagger is alive

    def on_scan(self, m):
        self.scan = (list(m.ranges), m.angle_min, m.angle_increment)

    def pose_at(self, stamp):
        try:
            tf = self.tfbuf.lookup_transform(self.map_frame, self.base_frame, Time(seconds=stamp),
                                             timeout=Duration(seconds=0.05))
        except Exception:
            try:      # image slightly newer than the newest TF: use the latest pose
                tf = self.tfbuf.lookup_transform(self.map_frame, self.base_frame, Time())
            except Exception:
                return None
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return tf.transform.translation.x, tf.transform.translation.y, yaw

    def on_boxes(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        if not d.get('boxes'):
            return
        pose = self.pose_at(d['stamp'])
        if pose is None:
            return
        p = dict(self.p, img_w=d['w'], img_h=d['h'])
        for b in d['boxes']:
            r = box_to_base(b, p)
            if r is None:
                continue
            xy, how = refine_with_lidar(r[:2], b, self.scan, p)
            mx, my = base_to_map(pose, xy)
            self.n_seen += 1
            self.get_logger().info('person seen -> map (%.1f, %.1f)' % (mx, my), throttle_duration_sec=2.0)
            self.tracker.update(d['stamp'], mx, my, b[4])
        for e in self.tracker.pop_events():
            self.get_logger().info(e)
        sv = self.tracker.survivors()
        if len(sv) != self.last_n:
            self.last_n = len(sv)
            self.save(sv)
            self.publish_markers()

    def save(self, sv):
        with open(self.out + '.json', 'w') as f:
            json.dump(dict(count=len(sv), survivors=sv), f, indent=2)
        with open(self.out + '.csv', 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['id', 'x_map_m', 'y_map_m', 'grid_box', 'sightings', 'first_seen_s'])
            for s in sv:
                w.writerow([s['id'], s['x'], s['y'], s['box'], s['sightings'], s['first_seen_s']])
        self.get_logger().info('saved %d survivor(s) -> %s.json / .csv' % (len(sv), self.out))

    def publish_markers(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for k, s in enumerate(self.tracker.survivors()):
            for kind, ident, scale, col, z in ((Marker.CYLINDER, 0, (0.4, 0.4, 0.05), (1.0, 0.1, 0.1, 0.9), 0.03),
                                                (Marker.TEXT_VIEW_FACING, 1, (0.0, 0.0, 0.45), (1.0, 1.0, 1.0, 1.0), 0.5)):
                m = Marker()
                m.header.frame_id = self.map_frame
                m.ns = 'survivors'
                m.id = k * 2 + ident
                m.type = kind
                m.action = Marker.ADD
                m.pose.position = Point(x=float(s['x']), y=float(s['y']), z=z)
                m.pose.orientation.w = 1.0
                m.scale.x, m.scale.y, m.scale.z = scale
                m.color.r, m.color.g, m.color.b, m.color.a = col
                if kind == Marker.TEXT_VIEW_FACING:
                    m.text = '%s  %s' % (s['id'], s['box'])
                arr.markers.append(m)
        self.pub.publish(arr)


def main():
    rclpy.init()
    node = SurvivorTagger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save(node.tracker.survivors())
    rclpy.shutdown()


if __name__ == '__main__':
    main()