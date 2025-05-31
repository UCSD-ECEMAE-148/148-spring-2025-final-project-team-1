#!/usr/bin/env python3
"""
Continuous Servo Test Code for Jetson Nano with Adafruit Driver Board
This code tests a continuous rotation servo motor connected through an Adafruit PWM driver board.

Continuous servos work differently from regular servos:
- They rotate continuously rather than moving to specific positions
- PWM controls speed and direction, not position
- ~1.5ms pulse = stop, <1.5ms = one direction, >1.5ms = other direction

Hardware Setup:
- Jetson Nano connected to Adafruit PCA9685 PWM driver board via I2C
- Continuous servo connected to one of the PWM channels on the driver board
- Power supply for servos (usually 5V or 6V depending on servo specs)

Required Libraries:
pip3 install adafruit-circuitpython-pca9685
"""

import time
import board
import busio
from adafruit_pca9685 import PCA9685

class ContinuousServoTester:
    def __init__(self, servo_channel=0):
        """
        Initialize the continuous servo tester
        
        Args:
            servo_channel (int): PWM channel number (0-15 on PCA9685)
        """
        print("Initializing Continuous Servo Tester...")
        
        # Create I2C bus - using default I2C pins on Jetson Nano
        self.i2c = busio.I2C(board.SCL, board.SDA)
        
        # Create PCA9685 PWM driver instance
        self.pca = PCA9685(self.i2c)
        
        # Set PWM frequency to 50Hz (standard for servos)
        self.pca.frequency = 50
        
        # Get the PWM channel for our servo
        self.servo_channel = servo_channel
        self.pwm_channel = self.pca.channels[servo_channel]
        
        # PWM values for continuous servo control
        # These values work for most continuous servos, but may need adjustment
        self.STOP_PULSE = 1.5      # milliseconds - servo stops
        self.MIN_PULSE = 1.0       # milliseconds - full speed one direction
        self.MAX_PULSE = 2.0       # milliseconds - full speed other direction
        
        # Convert milliseconds to duty cycle (0-65535 for PCA9685)
        self.STOP_DUTY = int((self.STOP_PULSE / 20.0) * 65535)
        self.MIN_DUTY = int((self.MIN_PULSE / 20.0) * 65535)
        self.MAX_DUTY = int((self.MAX_PULSE / 20.0) * 65535)
        
        print(f"Continuous servo initialized on channel {servo_channel}")
        print(f"Stop duty cycle: {self.STOP_DUTY}")
        print(f"Range: {self.MIN_DUTY} - {self.MAX_DUTY}")
        
        # Start with servo stopped
        self.stop()
    
    def set_speed(self, speed):
        """
        Set servo speed and direction
        
        Args:
            speed (float): Speed from -1.0 to 1.0
                          -1.0 = full speed clockwise
                           0.0 = stopped
                           1.0 = full speed counter-clockwise
        """
        # Clamp speed to valid range
        speed = max(-1.0, min(1.0, speed))
        
        # Convert speed to duty cycle
        if speed == 0:
            duty_cycle = self.STOP_DUTY
        elif speed > 0:
            # Positive speed: counter-clockwise
            duty_cycle = int(self.STOP_DUTY + (speed * (self.MAX_DUTY - self.STOP_DUTY)))
        else:
            # Negative speed: clockwise
            duty_cycle = int(self.STOP_DUTY + (speed * (self.STOP_DUTY - self.MIN_DUTY)))
        
        self.pwm_channel.duty_cycle = duty_cycle
        
        if speed == 0:
            print("Servo stopped")
        else:
            direction = "counter-clockwise" if speed > 0 else "clockwise"
            print(f"Servo spinning {direction} at {abs(speed)*100:.1f}% speed")
    
    def stop(self):
        """Stop the servo"""
        self.set_speed(0)
    
    def clockwise(self, speed=0.5):
        """
        Rotate clockwise at specified speed
        
        Args:
            speed (float): Speed from 0.0 to 1.0
        """
        self.set_speed(-abs(speed))
    
    def counter_clockwise(self, speed=0.5):
        """
        Rotate counter-clockwise at specified speed
        
        Args:
            speed (float): Speed from 0.0 to 1.0
        """
        self.set_speed(abs(speed))
    
    def speed_test(self):
        """
        Test different speeds in both directions
        """
        print("\n=== Speed Test ===")
        speeds = [0.2, 0.4, 0.6, 0.8, 1.0]
        
        for speed in speeds:
            print(f"\nTesting clockwise at {speed*100}% speed...")
            self.clockwise(speed)
            time.sleep(3)
            
            print(f"Testing counter-clockwise at {speed*100}% speed...")
            self.counter_clockwise(speed)
            time.sleep(3)
            
            print("Stopping...")
            self.stop()
            time.sleep(1)
    
    def direction_test(self):
        """
        Test basic direction changes
        """
        print("\n=== Direction Test ===")
        
        print("Spinning clockwise for 5 seconds...")
        self.clockwise(0.7)
        time.sleep(5)
        
        print("Stopping for 2 seconds...")
        self.stop()
        time.sleep(2)
        
        print("Spinning counter-clockwise for 5 seconds...")
        self.counter_clockwise(0.7)
        time.sleep(5)
        
        print("Stopping...")
        self.stop()
    
    def ramping_test(self):
        """
        Test gradual speed changes (ramping up and down)
        """
        print("\n=== Ramping Test ===")
        
        print("Ramping up clockwise...")
        for i in range(0, 101, 10):
            speed = i / 100.0
            self.clockwise(speed)
            time.sleep(0.5)
        
        print("Ramping down...")
        for i in range(100, -1, -10):
            speed = i / 100.0
            self.clockwise(speed)
            time.sleep(0.5)
        
        print("Stopping...")
        self.stop()
        time.sleep(1)
        
        print("Ramping up counter-clockwise...")
        for i in range(0, 101, 10):
            speed = i / 100.0
            self.counter_clockwise(speed)
            time.sleep(0.5)
        
        print("Ramping down...")
        for i in range(100, -1, -10):
            speed = i / 100.0
            self.counter_clockwise(speed)
            time.sleep(0.5)
        
        print("Stopping...")
        self.stop()
    
    def oscillation_test(self, duration=20):
        """
        Test rapid direction changes
        
        Args:
            duration (int): Test duration in seconds
        """
        print(f"\n=== Oscillation Test ({duration} seconds) ===")
        
        start_time = time.time()
        direction = 1
        
        while (time.time() - start_time) < duration:
            if direction == 1:
                self.clockwise(0.6)
            else:
                self.counter_clockwise(0.6)
            
            time.sleep(1)  # Change direction every second
            direction *= -1
        
        print("Stopping...")
        self.stop()
    
    def calibration_test(self):
        """
        Help calibrate the servo's stop position
        Some continuous servos need fine-tuning to find the exact stop point
        """
        print("\n=== Calibration Test ===")
        print("This test helps find the exact stop position for your servo")
        print("Watch the servo carefully - it should be completely stopped")
        
        # Test values around the nominal stop position
        test_values = [1.45, 1.47, 1.49, 1.50, 1.51, 1.53, 1.55]
        
        for pulse_ms in test_values:
            duty_cycle = int((pulse_ms / 20.0) * 65535)
            self.pwm_channel.duty_cycle = duty_cycle
            print(f"Testing {pulse_ms}ms pulse width (duty: {duty_cycle})")
            print("Is the servo stopped? (observe for 5 seconds)")
            time.sleep(5)
        
        print("Calibration test complete")
        print("Note the pulse width where the servo was most still")
        self.stop()
    
    def manual_control(self):
        """
        Interactive manual control mode
        """
        print("\n=== Manual Control Mode ===")
        print("Commands:")
        print("  cw <speed>  - Clockwise (e.g., 'cw 0.5')")
        print("  ccw <speed> - Counter-clockwise (e.g., 'ccw 0.8')")
        print("  stop        - Stop servo")
        print("  q           - Quit")
        print()
        
        while True:
            try:
                command = input("Enter command: ").strip().lower()
                
                if command == 'q':
                    break
                elif command == 'stop':
                    self.stop()
                elif command.startswith('cw'):
                    parts = command.split()
                    speed = float(parts[1]) if len(parts) > 1 else 0.5
                    self.clockwise(speed)
                elif command.startswith('ccw'):
                    parts = command.split()
                    speed = float(parts[1]) if len(parts) > 1 else 0.5
                    self.counter_clockwise(speed)
                else:
                    print("Unknown command. Use: cw <speed>, ccw <speed>, stop, or q")
                    
            except (ValueError, IndexError):
                print("Invalid command format. Speed should be a number between 0 and 1")
            except KeyboardInterrupt:
                break
    
    def cleanup(self):
        """
        Clean up resources and stop servo
        """
        print("\nCleaning up...")
        self.stop()
        time.sleep(1)
        self.pca.deinit()
        print("Cleanup completed")

def main():
    """
    Main function to run continuous servo tests
    """
    print("=== Continuous Servo Test Program ===")
    print("Make sure your continuous servo is connected to the Adafruit driver board")
    print("and the driver board is connected to the Jetson Nano via I2C\n")
    
    try:
        # Initialize servo tester
        # Change servo_channel if your servo is connected to a different channel
        tester = ContinuousServoTester(servo_channel=0)
        
        # Show menu
        while True:
            print("\n" + "="*50)
            print("Select a test:")
            print("1. Basic Direction Test")
            print("2. Speed Test")
            print("3. Ramping Test")
            print("4. Oscillation Test")
            print("5. Calibration Test")
            print("6. Manual Control")
            print("7. All Tests")
            print("8. Quit")
            
            choice = input("Enter choice (1-8): ").strip()
            
            if choice == '1':
                tester.direction_test()
            elif choice == '2':
                tester.speed_test()
            elif choice == '3':
                tester.ramping_test()
            elif choice == '4':
                duration = input("Enter test duration in seconds (default 20): ").strip()
                duration = int(duration) if duration.isdigit() else 20
                tester.oscillation_test(duration)
            elif choice == '5':
                tester.calibration_test()
            elif choice == '6':
                tester.manual_control()
            elif choice == '7':
                print("Running all tests...")
                tester.direction_test()
                tester.speed_test()
                tester.ramping_test()
                tester.oscillation_test(10)
            elif choice == '8':
                break
            else:
                print("Invalid choice. Please enter 1-8.")
        
    except Exception as e:
        print(f"Error occurred: {e}")
        print("Check your connections and make sure the I2C interface is enabled")
    
    finally:
        # Always cleanup
        try:
            tester.cleanup()
        except:
            pass

if __name__ == "__main__":
    main()
