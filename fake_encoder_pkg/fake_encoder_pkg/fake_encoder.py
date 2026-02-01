import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class FakeEncoder(Node):
    def __init__(self):
        super().__init__('fake_encoder')

        # Parameters
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('ticks_per_rev', 2048)
        self.declare_parameter('left_rps', 1.0)
        self.declare_parameter('right_rps', 1.0)

        self.publish_rate = self.get_parameter('publish_rate_hz').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.left_rps = self.get_parameter('left_rps').value
        self.right_rps = self.get_parameter('right_rps').value

        self.left_ticks = 0
        self.right_ticks = 0

        self.publisher = self.create_publisher(
            Int32MultiArray,
            '/wheel_ticks',
            10
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )

        self.get_logger().info('Fake encoder node started')

    def timer_callback(self):
        dt = 1.0 / self.publish_rate

        self.left_ticks += int(self.left_rps * self.ticks_per_rev * dt)
        self.right_ticks += int(self.right_rps * self.ticks_per_rev * dt)

        msg = Int32MultiArray()
        msg.data = [self.left_ticks, self.right_ticks]

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = FakeEncoder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
