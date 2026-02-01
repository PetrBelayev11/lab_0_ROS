#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32
import math


class OdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odom')

        self.create_subscription(
            Int32,
            '/wheel_ticks',
            self.encoder_callback,
            10
        )

        self.publisher = self.create_publisher(Odometry, '/odom', 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_ticks = None
        self.last_time = self.get_clock().now()

        self.ticks_per_meter = 1000.0  # adjust if needed

    def encoder_callback(self, msg):
        now = self.get_clock().now()

        if self.last_ticks is None:
            self.last_ticks = msg.data
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        delta_ticks = msg.data - self.last_ticks
        distance = delta_ticks / self.ticks_per_meter

        # Straight-line motion (no steering info available)
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation = q

        odom.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 9999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 9999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 9999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        odom.twist.twist.linear.x = distance / dt
        odom.twist.twist.angular.z = 0.0

        self.publisher.publish(odom)

        self.last_ticks = msg.data
        self.last_time = now


def main():
    rclpy.init()
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
