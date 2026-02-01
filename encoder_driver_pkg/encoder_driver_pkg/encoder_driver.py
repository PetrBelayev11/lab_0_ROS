import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
import math


class EncoderDriver(Node):
    def __init__(self):
        super().__init__('encoder_driver')

        self.declare_parameter('ticks_per_rev', 2048)

        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value

        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/wheel_ticks',
            self.ticks_callback,
            10
        )

        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.get_logger().info('Encoder driver node started')

    def ticks_callback(self, msg):
        left_ticks = msg.data[0]
        right_ticks = msg.data[1]

        left_angle = 2.0 * math.pi * left_ticks / self.ticks_per_rev
        right_angle = 2.0 * math.pi * right_ticks / self.ticks_per_rev

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [left_angle, right_angle]

        self.publisher.publish(joint_state)


def main():
    rclpy.init()
    node = EncoderDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
