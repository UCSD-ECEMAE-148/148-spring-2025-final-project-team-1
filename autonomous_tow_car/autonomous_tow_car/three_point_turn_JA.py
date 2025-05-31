#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class ThreePointTurn(Node):
    def __init__(self):
        super().__init__('three_point_turn')
        
        # Create ROS2 publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Movement parameters - adjust these based on your car's characteristics
        self.forward_speed = 0.3      # Linear velocity for forward movement
        self.reverse_speed = -0.3     # Linear velocity for reverse movement
        self.turn_speed = 1.0         # Angular velocity for turning
        
        # Timing parameters - adjust based on your car's size and speed
        self.forward_duration = 2.0   # Time to drive forward in seconds
        self.turn_duration = 2.0      # Time to turn in seconds
        self.reverse_duration = 2.0   # Time to reverse in seconds
        
        # State tracking
        self.current_step = 0
        self.step_start_time = None
        self.maneuver_complete = False
        
        # Start the maneuver
        self.timer = self.create_timer(0.1, self.execute_maneuver)  # 10Hz update rate
        
        self.get_logger().info('Starting 3-point turn maneuver')
    
    def publish_twist(self, linear_x=0.0, angular_z=0.0):
        """Publish a Twist message with the given linear and angular velocities"""
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_x
        cmd_vel_msg.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd_vel_msg)
    
    def stop_robot(self):
        """Stop all robot movement"""
        self.publish_twist(0.0, 0.0)
        self.get_logger().info('Robot stopped')
    
    def execute_maneuver(self):
        """Execute the 3-point turn maneuver step by step"""
        
        if self.maneuver_complete:
            return
        
        # Initialize step timer
        if self.step_start_time is None:
            self.step_start_time = time.time()
        
        elapsed_time = time.time() - self.step_start_time
        
        # Step 0: Drive forward while turning left
        if self.current_step == 0:
            if elapsed_time < self.forward_duration:
                self.publish_twist(self.forward_speed, self.turn_speed)
                self.get_logger().info(f'Step 1: Forward + Left turn ({elapsed_time:.1f}s)')
            else:
                self.stop_robot()
                self.current_step = 1
                self.step_start_time = None
                time.sleep(0.5)  # Brief pause between steps
        
        # Step 1: Reverse while turning right
        elif self.current_step == 1:
            if elapsed_time < self.reverse_duration:
                self.publish_twist(self.reverse_speed, -self.turn_speed)
                self.get_logger().info(f'Step 2: Reverse + Right turn ({elapsed_time:.1f}s)')
            else:
                self.stop_robot()
                self.current_step = 2
                self.step_start_time = None
                time.sleep(0.5)  # Brief pause between steps
        
        # Step 2: Drive forward to complete the turn
        elif self.current_step == 2:
            if elapsed_time < self.forward_duration:
                self.publish_twist(self.forward_speed, 0.0)
                self.get_logger().info(f'Step 3: Forward straight ({elapsed_time:.1f}s)')
            else:
                self.stop_robot()
            time.sleep(0.5)
        self.stop_robot()
        self.maneuver_complete = True
        self.get_logger().info('3-point turn maneuver completed!')
        self.timer.cancel()
    
    def emergency_stop(self):
        """Emergency stop function"""
        self.stop_robot()
        self.maneuver_complete = True
        if hasattr(self, 'timer'):
            self.timer.cancel()
        self.get_logger().warn('Emergency stop activated!')

def main(args=None):
    rclpy.init(args=args)
    three_point_turn = ThreePointTurn()
    
    try:
        rclpy.spin(three_point_turn)
    except KeyboardInterrupt:
        three_point_turn.emergency_stop()
    finally:
        three_point_turn.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
                
