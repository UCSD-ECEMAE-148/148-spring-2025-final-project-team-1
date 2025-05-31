import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time
import sys

class ThreePointTurnNode(Node):
    def __init__(self):
        super().__init__('three_point_turn')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Bool,
            'damsel_found',
            self.damsel_callback,
            10
        )
        self.triggered = False

    def damsel_callback(self, msg):
        if msg.data and not self.triggered:
            self.get_logger().info("Received 'damsel_found' signal. Executing three-point turn.")
            self.triggered = True
            self.execute_turn()

    def publish_cmd(self, linear_x, angular_z, duration):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_x
        cmd_vel_msg.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd_vel_msg)
        time.sleep(duration)
        self.stop_robot()

    def stop_robot(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def execute_turn(self):
        self.get_logger().info("Segment 1: Reverse left and straighten...")
        for angle in [-1.0, -0.7, -0.4, -0.1, 0.0]:
            self.publish_cmd(-0.3, angle, 0.5)

        self.get_logger().info("Segment 2: Forward straight...")
        self.publish_cmd(0.3, 0.0, 2.0)

        self.get_logger().info("Segment 3: Reverse left and straighten again...")
        for angle in [-1.0, -0.7, -0.4, -0.1, 0.0]:
            self.publish_cmd(-0.3, angle, 0.5)

        self.get_logger().info("Three-point turn completed.")

    def run_test_mode(self):
        self.get_logger().info("Running in TEST MODE...")
        self.execute_turn()


def main(args=None):
    rclpy.init(args=args)
    node = ThreePointTurnNode()

    if '--test' in sys.argv:
        node.run_test_mode()
    else:
        rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

