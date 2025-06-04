#!/usr/bin/env python3
"""
ROS2 Node for 1080 Degree Rotation Test
Rotates counter-clockwise 1080°, pauses 5 seconds, then rotates clockwise 1080° back.

Based on your working servo configuration:
- 90° = Stop
- 0° = Fast clockwise  
- 180° = Fast counter-clockwise
- Pulse width: 1000-2182 microseconds

ROS2 Setup:
- Service: ~/start_rotation (triggers the rotation sequence)
- Topic: ~/rotation_status (publishes current status)

Required: 
- pip3 install adafruit-circuitpython-servokit
- ROS2 installed
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
import time
import threading
from geometry_msgs.msg import Twist
from adafruit_servokit import ServoKit

class ServoRotationNode(Node):
    def __init__(self):
        super().__init__('servo_rotation_node')
        
        # Initialize ServoKit
        self.kit = ServoKit(channels=16)
        self.servo = self.kit.servo[0]  # Using channel 0
        self.servo.set_pulse_width_range(1000, 2182)
        
        # Servo control angles based on your test results
        self.STOP = 90
        self.COUNTER_CLOCKWISE = 135  # Medium speed counter-clockwise
        self.CLOCKWISE = 45           # Medium speed clockwise
        
        # Timing for 1080 degrees (3 full rotations)
        self.ROTATION_TIME_1080 = 13.0  # seconds
        self.ROTATION_TIME_540 = 6.0
      
        # ROS2 Service to start rotation sequence
        self.rotation_service = self.create_service(
            Trigger, 
            'start_rotation', 
            self.start_rotation_callback
        )
        
        # ROS2 Publisher for status updates
        self.status_publisher = self.create_publisher(
            String, 
            'rotation_status', 
            10
        )
        
        # Status tracking
        self.is_rotating = False
        self.current_status = "Ready"
        
        # Timer for periodic status publishing
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Initialize servo to stop position
        self.servo.angle = self.STOP
        
        self.get_logger().info('Servo Rotation Node initialized')
        self.get_logger().info('Call service: ros2 service call /servo_rotation_node/start_rotation std_srvs/srv/Trigger')
        self.get_logger().info('Monitor status: ros2 topic echo /servo_rotation_node/rotation_status')

        # UCSD RoboCar uses /cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # UCSD RoboCar specific parameters
        # Note: Some UCSD RoboCars have reversed polarity
        self.forward_speed = -0.3   # m/s (negative for forward on this car)
        self.forward_duration = 5.0      # Duration for straight forward

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
    
    def publish_status(self):
        """Publish current rotation status"""
        msg = String()
        msg.data = self.current_status
        self.status_publisher.publish(msg)
    
    def start_rotation_callback(self, request, response):
        """Service callback to start the rotation sequence"""
        if self.is_rotating:
            response.success = False
            response.message = "Rotation sequence already in progress"
            return response
        
        # Start rotation in separate thread to avoid blocking service
        rotation_thread = threading.Thread(target=self.execute_rotation_sequence)
        rotation_thread.daemon = True
        rotation_thread.start()
        
        response.success = True
        response.message = "1080° rotation sequence started"
        return response
    
    def execute_rotation_sequence(self):
        """Execute the complete 1080° rotation sequence"""
        self.is_rotating = True
        
        try:
            self.get_logger().info('Starting 1080° rotation sequence...')
            
            # Initial stop
            self.current_status = "Initializing - servo stop"
            self.servo.angle = self.STOP
            time.sleep(2)
            
            # Step 1: Rotate counter-clockwise 1080 degrees
            self.current_status = "Step 1/4: Rotating counter-clockwise 1080°"
            self.get_logger().info('Step 1: Rotating COUNTER-CLOCKWISE 1080 degrees...')
            self.servo.angle = self.COUNTER_CLOCKWISE
            
            # Countdown during rotation
            for i in range(int(self.ROTATION_TIME_1080)):
                if not rclpy.ok():  # Check if ROS is shutting down
                    break
                remaining = int(self.ROTATION_TIME_1080) - i
                self.current_status = f"Step 1/4: Counter-clockwise rotation - {remaining}s remaining"
                time.sleep(1)
            
            # Step 2: Stop and pause
            self.current_status = "Step 2/4: Stopping and pausing"
            self.get_logger().info('Step 2: Stopping and pausing for 5 seconds...')
            self.servo.angle = self.STOP
            
            for i in range(5):
                if not rclpy.ok():
                    break
                remaining = 5 - i
                self.current_status = f"Step 2/4: Pausing - {remaining}s remaining"
                time.sleep(1)
            
            # Step 3: Rotate clockwise 540 degrees back
            self.current_status = "Step 3/4: Rotating clockwise 1080°"
            self.get_logger().info('Step 3: Rotating CLOCKWISE 1080 degrees back...')
            self.servo.angle = self.CLOCKWISE
            
            # Countdown during return rotation
            for i in range(int(self.ROTATION_TIME_540)):
                if not rclpy.ok():
                    break
                remaining = int(self.ROTATION_TIME_540) - i
                self.current_status = f"Step 3/4: Clockwise rotation - {remaining}s remaining"
                time.sleep(1)
              
            # Step 4: Drive forward
            self.current_status = "Step 4: Driving forward"
            self.get_logger().info('Step 4: Driving forward...')
            self.publish_twist(self.forward_speed, 0.0)

            #Countdown during drive forward
            for i in range(int(self.ROTATION_TIME_540)):
            if not rclpy.ok():
                    break
              remaining = int(self.ROTATION_TIME_540) - i
              self.current_status = f"Step 4: Drive forward - {remaining}s remaining"
              time.sleep(1)  # Brief pause between steps
          
            self.stop_robot()

            # Step 5: Stop and pause
            self.current_status = "Step 5: Stopping and pausing"
            self.get_logger().info('Step 5: Stopping and pausing for 5 seconds...')
            self.servo.angle = self.STOP
            
            for i in range(5):
                if not rclpy.ok():
                    break
                remaining = 5 - i
                self.current_status = f"Step 5: Pausing - {remaining}s remaining"
                time.sleep(1)
              
            # Step 6: Rotate counter-clockwise 540 degrees
            self.current_status = "Step 6: Rotating counter-clockwise 540°"
            self.get_logger().info('Step 6: Rotating COUNTER-CLOCKWISE 540 degrees...')
            self.servo.angle = self.COUNTER_CLOCKWISE
            
            # Countdown during rotation
            for i in range(int(self.ROTATION_TIME_540)):
                if not rclpy.ok():  # Check if ROS is shutting down
                    break
                remaining = int(self.ROTATION_TIME_540) - i
                self.current_status = f"Step 6: Counter-clockwise rotation - {remaining}s remaining"
                time.sleep(1)

            # Step 7: Drive forward
            self.current_status = "Step 8: Driving forward"
            self.get_logger().info('Step 8: Driving forward...')
            self.publish_twist(self.forward_speed, 0.0)

            #Countdown during drive forward
            for i in range(int(self.ROTATION_TIME_540)):
            if not rclpy.ok():
                    break
              remaining = int(self.ROTATION_TIME_540) - i
              self.current_status = f"Step 4: Drive forward - {remaining}s remaining"
              time.sleep(1)  # Brief pause between steps
              

            # Step 8: Rotate clockwise 1080 degrees back
            self.current_status = "Step 7: Rotating clockwise 1080°"
            self.get_logger().info('Step 7: Rotating CLOCKWISE 1080 degrees back...')
            self.servo.angle = self.CLOCKWISE
            
            # Countdown during return rotation
            for i in range(int(self.ROTATION_TIME_1080)):
                if not rclpy.ok():
                    break
                remaining = int(self.ROTATION_TIME_1080) - i
                self.current_status = f"Step 7: Clockwise rotation - {remaining}s remaining"
                time.sleep(1)
            
            # Step 9: Final stop
            self.current_status = "Step 9: Final stop"
            self.get_logger().info('Step 9: Final stop')
            self.servo.angle = self.STOP
            time.sleep(2)
            
            self.current_status = "Completed - sequence finished"
            self.get_logger().info('Sequence completed!')
            
        except Exception as e:
            self.get_logger().error(f'Error during rotation: {str(e)}')
            self.current_status = f"Error: {str(e)}"
            self.servo.angle = self.STOP  # Safety stop
        
        finally:
            self.is_rotating = False
            # After 5 seconds, reset status to ready
            time.sleep(5)
            if not self.is_rotating:  # Only reset if no new rotation started
                self.current_status = "Ready"
    
    def cleanup(self):
        """Cleanup function called on shutdown"""
        self.get_logger().info('Shutting down - stopping servo')
        try:
            self.servo.angle = self.STOP
            time.sleep(1)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    servo_node = ServoRotationNode()
    
    try:
        rclpy.spin(servo_node)
    except KeyboardInterrupt:
        servo_node.get_logger().info('Keyboard interrupt received')
    finally:
        servo_node.cleanup()
        servo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
