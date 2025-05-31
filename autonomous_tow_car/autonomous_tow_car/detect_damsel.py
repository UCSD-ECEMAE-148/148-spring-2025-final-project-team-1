import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class DamselDetector(Node):
    def __init__(self):
        super().__init__('damsel_detector')
        self.publisher_ = self.create_publisher(Bool, 'damsel_found', 10)
        self.timer = self.create_timer(1.0, self.publish_signal)
        self.get_logger().info("DamselDetector node started!")

    def publish_signal(self):
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
        self.get_logger().info("Published: True")

def main(args=None):
    rclpy.init(args=args)
    node = DamselDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
