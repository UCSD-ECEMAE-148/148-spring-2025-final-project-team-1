#!/usr/bin/env python3
"""
Simple 540 Degree Rotation Test for Continuous Servo
Rotates counter-clockwise 540°, pauses 5 seconds, then rotates clockwise 540° back.

Based on your working servo configuration:
- 90° = Stop
- 0° = Fast clockwise  
- 180° = Fast counter-clockwise
- Pulse width: 1000-2182 microseconds

Required: pip3 install adafruit-circuitpython-servokit
"""

import time
from adafruit_servokit import ServoKit

# Initialize ServoKit
kit = ServoKit(channels=16)
servo = kit.servo[0]  # Using channel 0, change if needed
servo.set_pulse_width_range(1000, 2182)

# Servo control angles based on your test results
STOP = 90
COUNTER_CLOCKWISE = 135  # Medium speed counter-clockwise
CLOCKWISE = 45           # Medium speed clockwise

# Timing for 540 degrees (1.5 full rotations)
# Adjust this value based on your servo's actual speed
ROTATION_TIME_1080 = 13.0  
ROTATION_TIME_540 = 6.0

def main():
    print("Starting 1080° rotation test...")
    print("Make sure servo is connected to channel 0")
    print()
    
    try:
        # Start with servo stopped
        print("Initial stop...")
        servo.angle = STOP
        time.sleep(2)
        
        # Step 1: Rotate counter-clockwise 540 degrees
        print("Step 1: Rotating COUNTER-CLOCKWISE 540 degrees...")
        servo.angle = COUNTER_CLOCKWISE
        time.sleep(ROTATION_TIME_540)
        
        # Step 2: Final stop
        print("Step 4: Final stop")
        servo.angle = STOP
        time.sleep(2)
        
        print("hook_release_pt1 complete!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted - stopping servo")
        servo.angle = STOP
    except Exception as e:
        print(f"Error: {e}")
        servo.angle = STOP

if __name__ == "__main__":
    main()
