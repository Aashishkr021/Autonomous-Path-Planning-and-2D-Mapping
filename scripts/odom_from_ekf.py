#!/usr/bin/env python3
"""Odometry from ArduPilot's EKF3 (IMU + optical flow + rangefinder) instead of Gazebo ground truth.

Reads MAVLink LOCAL_POSITION_NED + ATTITUDE, converts NED/FRD -> ENU/FLU and publishes
  - /odom_ekf  (nav_msgs/Odometry)
  - TF odom -> base_link
slam_toolbox then uses this as its motion prior and corrects the drift with lidar scan matching
(map -> odom). Remove the '/tf' entry from config/bridge.yaml first, or two sources fight over odom->base_link.
Run:  python3 scripts/odom_from_ekf.py --ros-args -p use_sim_time:=true
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
from pymavlink import mavutil

R_ENU_NED = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)
R_FRD_FLU = np.diag([1.0, -1.0, -1.0])


class EkfOdom(Node):
    def __init__(self):
        super().__init__('odom_from_ekf')
        self.declare_parameter('mavlink', 'udp:127.0.0.1:14552')   # use a SEPARATE --out port
        self.m = mavutil.mavlink_connection(self.get_parameter('mavlink').value)
        self.get_logger().info('waiting for MAVLink heartbeat ...')
        self.m.wait_heartbeat()
        for mid, hz in ((32, 20), (30, 20)):                       # LOCAL_POSITION_NED, ATTITUDE
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                         mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                         mid, 1e6 / hz, 0, 0, 0, 0, 0)
        self.att = None
        self.pub = self.create_publisher(Odometry, '/odom_ekf', 10)
        self.tf = TransformBroadcaster(self)
        self.create_timer(0.01, self.pump)

    def pump(self):
        while True:
            msg = self.m.recv_match(type=['ATTITUDE', 'LOCAL_POSITION_NED'], blocking=False)
            if msg is None:
                return
            if msg.get_type() == 'ATTITUDE':
                self.att = (msg.roll, msg.pitch, msg.yaw)
            elif self.att is not None:
                self.publish(msg)

    def publish(self, p):
        roll, pitch, yaw = self.att
        r_ned_frd = R.from_euler('ZYX', [yaw, pitch, roll]).as_matrix()
        q = R.from_matrix(R_ENU_NED @ r_ned_frd @ R_FRD_FLU).as_quat()          # x y z w
        now = self.get_clock().now().to_msg()
        o = Odometry()
        o.header.stamp = now; o.header.frame_id = 'odom'; o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = p.y, p.x, -p.z
        (o.pose.pose.orientation.x, o.pose.pose.orientation.y,
         o.pose.pose.orientation.z, o.pose.pose.orientation.w) = q
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = p.vy, p.vx, -p.vz
        self.pub.publish(o)
        t = TransformStamped()
        t.header.stamp = now; t.header.frame_id = 'odom'; t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.y, p.x, -p.z
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(EkfOdom())


if __name__ == '__main__':
    main()
