#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DriveForward(Node):
    def __init__(self):
        super().__init__('drive_forward')
        
        # UCSD RoboCar uses /cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # UCSD RoboCar specific parameters (same as your three-point turn)
        self.forward_speed = 0.3   # m/s (negative for forward on this car)
        
        # Timing parameter
        self.drive_duration = 3.0    # Drive forward for 3 seconds
        
        # State tracking
        self.drive_complete = False
        self.start_time = None
        
        # Wait for initialization
        self.get_logger().info('Starting drive forward for 3 seconds')
        time.sleep(2.0)
        
        # Start driving
        self.timer = self.create_timer(0.1, self.execute_drive)
    
    def publish_twist(self, linear_x=0.0, angular_z=0.0):
        """Publish Twist message"""
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_x
        cmd_vel_msg.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
    def stop_robot(self):
        """Stop all robot movement"""
        self.publish_twist(0.0, 0.0)
        self.get_logger().info('Robot stopped')
    
    def execute_drive(self):
        """Execute the drive forward"""
        
        if self.drive_complete:
            return
        
        # Initialize timer
        if self.start_time is None:
            self.start_time = time.time()
        
        elapsed_time = time.time() - self.start_time
        
        # Drive forward for 3 seconds
        if elapsed_time < self.drive_duration:
            self.publish_twist(self.forward_speed, 0.0)
            remaining = self.drive_duration - elapsed_time
            self.get_logger().info(f'Driving forward ({remaining:.1f}s remaining)')
        else:
            self.stop_robot()
            self.drive_complete = True
            self.get_logger().info('Drive forward completed!')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    drive_forward = DriveForward()
    
    try:
        rclpy.spin(drive_forward)
    except KeyboardInterrupt:
        drive_forward.stop_robot()
    finally:
        drive_forward.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
