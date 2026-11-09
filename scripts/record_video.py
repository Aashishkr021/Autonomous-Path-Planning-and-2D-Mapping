#!/usr/bin/env python3
"""Records /camera/detections_image (or /camera/image_raw) to videos/flight.mp4. Ctrl+C to finish."""
import os, sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPIC = sys.argv[1] if len(sys.argv) > 1 else '/camera/image_raw'
OUT = os.path.join(os.environ.get('NIDAR_HOME', os.path.expanduser('~/nidar_airmouse')), 'videos', 'flight.mp4')


class Rec(Node):
    def __init__(self):
        super().__init__('record_video')
        self.out = None
        self.create_subscription(Image, TOPIC, self.cb, 10)

    def cb(self, msg):
        rgb = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3)
        frame = np.ascontiguousarray(rgb[:, :, ::-1])
        if self.out is None:
            self.out = cv2.VideoWriter(OUT, cv2.VideoWriter_fourcc(*'mp4v'), 15, (msg.width, msg.height))
            self.get_logger().info('Recording %s -> %s' % (TOPIC, OUT))
        self.out.write(frame)


rclpy.init()
n = Rec()
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
if n.out:
    n.out.release()
