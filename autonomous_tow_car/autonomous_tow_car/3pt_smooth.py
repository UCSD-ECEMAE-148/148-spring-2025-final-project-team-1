#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class ThreePointTurn(Node):
    def __init__(self):
        super().__init__('three_point_turn')
        
        # UCSD RoboCar uses /cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # UCSD RoboCar specific parameters
        self.max_forward_speed = 0.3   # m/s
        self.max_reverse_speed = -0.3  # m/s
        self.max_turn_speed = 0.5      # rad/s
        
        # Acceleration parameters for smooth motion
        self.linear_accel = 0.2        # m/s^2
        self.angular_accel = 0.3       # rad/s^2
        
        # Current velocities (for smooth ramping)
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Target velocities
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        # Timing parameters
        self.forward_duration = 2.5    # Increased for smoother motion
        self.reverse_duration = 2.5
        self.pause_duration = 0.3      # Shorter pause between steps
        
        # State tracking
        self.current_step = 0
        self.step_start_time = None
        self.maneuver_complete = False
        self.is_pausing = False
        
        # Control loop frequency
        self.control_frequency = 20.0  # Hz
        self.dt = 1.0 / self.control_frequency
        
        # Wait for initialization
        self.get_logger().info('Initializing smooth 3-point turn...')
        time.sleep(2.0)
        
        # Start the maneuver with higher frequency for smoother control
        self.timer = self.create_timer(self.dt, self.execute_maneuver)
        self.get_logger().info('Starting smooth 3-point turn maneuver')
    
    def smooth_velocity_update(self):
        """Smoothly ramp velocities toward targets"""
        # Linear velocity ramping
        if abs(self.target_linear - self.current_linear) > 0.01:
            linear_diff = self.target_linear - self.current_linear
            linear_step = self.linear_accel * self.dt
            
            if abs(linear_diff) < linear_step:
                self.current_linear = self.target_linear
            else:
                self.current_linear += linear_step if linear_diff > 0 else -linear_step
        
        # Angular velocity ramping
        if abs(self.target_angular - self.current_angular) > 0.01:
            angular_diff = self.target_angular - self.current_angular
            angular_step = self.angular_accel * self.dt
            
            if abs(angular_diff) < angular_step:
                self.current_angular = self.target_angular
            else:
                self.current_angular += angular_step if angular_diff > 0 else -angular_step
    
    def publish_twist(self):
        """Publish current smoothed velocities"""
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.current_linear
        cmd_vel_msg.angular.z = self.current_angular
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
    def set_target_velocities(self, linear, angular):
        """Set target velocities for smooth ramping"""
        self.target_linear = linear
        self.target_angular = angular
    
    def stop_robot(self):
        """Gradually stop the robot"""
        self.set_target_velocities(0.0, 0.0)
        self.get_logger().info('Stopping robot smoothly...')
    
    def execute_maneuver(self):
        """Execute the smooth 3-point turn maneuver"""
        
        if self.maneuver_complete:
            return
        
        # Always update velocities smoothly
        self.smooth_velocity_update()
        self.publish_twist()
        
        # Initialize step timer
        if self.step_start_time is None and not self.is_pausing:
            self.step_start_time = time.time()
        
        # Handle pausing between steps
        if self.is_pausing:
            if time.time() - self.pause_start_time > self.pause_duration:
                self.is_pausing = False
                self.step_start_time = None
            return
        
        elapsed_time = time.time() - self.step_start_time
        
        # Step 0: Forward while turning left (wider arc)
        if self.current_step == 0:
            if elapsed_time < self.forward_duration:
                # Gradual turn - less aggressive
                turn_factor = min(1.0, elapsed_time / 1.0)  # Ramp up turn over 1 second
                self.set_target_velocities(
                    self.max_forward_speed,
                    self.max_turn_speed * 0.7 * turn_factor  # 70% of max turn for wider arc
                )
                
                if int(elapsed_time * 10) % 5 == 0:
                    self.get_logger().info(f'Step 1: Forward + Left ({elapsed_time:.1f}s)')
            else:
                self.stop_robot()
                self.current_step = 1
                self.is_pausing = True
                self.pause_start_time = time.time()
        
        # Step 1: Reverse while turning right
        elif self.current_step == 1:
            if elapsed_time < self.reverse_duration:
                # Gradual turn
                turn_factor = min(1.0, elapsed_time / 1.0)
                self.set_target_velocities(
                    self.max_reverse_speed,
                    -self.max_turn_speed * 0.7 * turn_factor
                )
                
                if int(elapsed_time * 10) % 5 == 0:
                    self.get_logger().info(f'Step 2: Reverse + Right ({elapsed_time:.1f}s)')
            else:
                self.stop_robot()
                self.current_step = 2
                self.is_pausing = True
                self.pause_start_time = time.time()
        
        # Step 2: Forward straight (with slight correction if needed)
        elif self.current_step == 2:
            if elapsed_time < self.forward_duration:
                # Start straight, then allow slight correction
                correction = 0.0
                if elapsed_time > 0.5:  # After initial straight movement
                    correction = 0.05  # Very slight turn if needed
                
                self.set_target_velocities(self.max_forward_speed, correction)
                
                if int(elapsed_time * 10) % 5 == 0:
                    self.get_logger().info(f'Step 3: Forward straight ({elapsed_time:.1f}s)')
            else:
                self.stop_robot()
                
                # Wait for complete stop
                if abs(self.current_linear) < 0.01 and abs(self.current_angular) < 0.01:
                    self.maneuver_complete = True
                    self.get_logger().info('Smooth 3-point turn completed!')
                    self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    three_point_turn = ThreePointTurn()
    
    try:
        rclpy.spin(three_point_turn)
    except KeyboardInterrupt:
        # Ensure smooth stop on interrupt
        three_point_turn.stop_robot()
        time.sleep(1.0)  # Allow time for smooth stop
    finally:
        three_point_turn.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
