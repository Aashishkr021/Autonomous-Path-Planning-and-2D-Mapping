#!/usr/bin/env python3
"""YOLO person detector: /camera/image_raw -> /camera/detections_image + /person_detections."""
import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


class PersonDetector(Node):
    def __init__(self):
        super().__init__('yolo_person')
        self.declare_parameter('model', 'yolov5nu.pt')   # YOLOv5n (nano). Other choice: yolov8n.pt
        self.declare_parameter('conf', 0.35)
        name = self.get_parameter('model').value
        self.conf = float(self.get_parameter('conf').value)
        self.model = YOLO(name)           # looks in the current folder first, downloads only if missing
        self.get_logger().info('YOLO model: %s  (conf >= %.2f)' % (name, self.conf))
        self.create_subscription(Image, '/camera/image_raw', self.cb, 1)
        self.pub_img = self.create_publisher(Image, '/camera/detections_image', 1)
        self.pub_txt = self.create_publisher(String, '/person_detections', 10)
        self.pub_box = self.create_publisher(String, '/person_boxes', 10)   # machine-readable, used by survivor_tagger
        self.get_logger().info('YOLO ready, waiting for images')

    def cb(self, msg):
        rgb = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3)
        bgr = np.ascontiguousarray(rgb[:, :, ::-1])
        res = self.model.predict(bgr, classes=[0], conf=self.conf, verbose=False)[0]  # 0 = person
        n = len(res.boxes)
        out = Image()
        out.header = msg.header
        out.height, out.width = msg.height, msg.width
        out.encoding = 'rgb8'
        out.step = msg.width * 3
        out.data = np.ascontiguousarray(res.plot()[:, :, ::-1]).tobytes()
        self.pub_img.publish(out)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        boxes = [[*[float(v) for v in b.xyxy[0].tolist()], float(b.conf[0])] for b in res.boxes]
        self.pub_box.publish(String(data=json.dumps(dict(stamp=stamp, w=msg.width, h=msg.height, boxes=boxes))))
        if n:
            parts = []
            for b in res.boxes:
                x1, y1, x2, y2 = b.xyxy[0].tolist()
                parts.append('conf=%.2f center=(%.0f,%.0f)' % (float(b.conf[0]), (x1 + x2) / 2, (y1 + y2) / 2))
            text = 'persons=%d  ' % n + ' | '.join(parts)
            self.pub_txt.publish(String(data=text))
            self.get_logger().info(text)


def main():
    rclpy.init()
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()