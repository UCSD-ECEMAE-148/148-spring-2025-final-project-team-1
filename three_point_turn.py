#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class ThreePointTurn(Node):
    def __init__(self):
        super().__init__('three_point_turn')
        
        # UCSD RoboCar uses /cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # UCSD RoboCar specific parameters
        # Note: Some UCSD RoboCars have reversed polarity
        self.forward_speed = 0.3   # m/s (negative for forward on this car)
        self.reverse_speed = -0.3    # m/s (positive for reverse on this car)
        self.turn_speed = -0.4      # rad/s (negative for right turn)
        
        # Timing parameters
        self.reverse_duration_1 = 2.8    # Duration for first reverse+turn
        self.forward_duration = 3.8      # Duration for straight forward
        self.reverse_duration_2 = 2.75    # Duration for second reverse+turn
        self.reverse_straight_duration = 0.1  # Duration for straight reverse
        
        # State tracking
        self.current_step = 0
        self.step_start_time = None
        self.maneuver_complete = False
        
        # Wait for initialization
        self.get_logger().info('Starting maneuver: Reverse-Right, Forward, Reverse-Right, Reverse-Straight')
        time.sleep(2.0)
        
        # Start the maneuver
        self.timer = self.create_timer(0.1, self.execute_maneuver)
    
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
    
    def execute_maneuver(self):
        """Execute the maneuver"""
        
        if self.maneuver_complete:
            return
        
        # Initialize step timer
        if self.step_start_time is None:
            self.step_start_time = time.time()
        
        elapsed_time = time.time() - self.step_start_time
        
        # Step 0: Reverse while turning right
        if self.current_step == 0:
            if elapsed_time < self.reverse_duration_1:
                self.publish_twist(self.reverse_speed, self.turn_speed)
                self.get_logger().info(f'Step 1: Reverse + Right turn ({elapsed_time:.1f}s)')
            else:
                self.stop_robot()
                self.current_step = 1
                self.step_start_time = None
                time.sleep(1.5)  # Brief pause between steps
        
        # Step 1: Forward straight
        elif self.current_step == 1:
            if elapsed_time < self.forward_duration:
                self.publish_twist(self.forward_speed, 0.0)
                self.get_logger().info(f'Step 2: Forward straight ({elapsed_time:.1f}s)')
            else:
                self.stop_robot()
                self.current_step = 2
                self.step_start_time = None
                time.sleep(1.5)  # Brief pause between steps
        
        # Step 2: Reverse while turning right
        elif self.current_step == 2:
            if elapsed_time < self.reverse_duration_2:
                self.publish_twist(self.reverse_speed, self.turn_speed)
                self.get_logger().info(f'Step 3: Reverse + Right turn ({elapsed_time:.1f}s)')
            else:
                self.stop_robot()
                self.current_step = 3
                self.step_start_time = None
                time.sleep(1.5)  # Brief pause between steps
        
        # Step 3: Reverse straight
        elif self.current_step == 3:
            if elapsed_time < self.reverse_straight_duration:
                self.publish_twist(self.reverse_speed, 0.0)
                self.get_logger().info(f'Step 4: Reverse straight ({elapsed_time:.1f}s)')
            else:
                self.stop_robot()
                self.maneuver_complete = True
                self.get_logger().info('Maneuver completed!')
                self.timer.cancel()
                time.sleep(1.5)

def main(args=None):
    rclpy.init(args=args)
    three_point_turn = ThreePointTurn()
    
    try:
        rclpy.spin(three_point_turn)
    except KeyboardInterrupt:
        three_point_turn.stop_robot()
    finally:
        three_point_turn.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
