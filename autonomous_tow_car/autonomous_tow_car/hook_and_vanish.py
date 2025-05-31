#!/usr/bin/env python3
import time
import board
from adafruit_pca9685 import PCA9685

# ── Setup ────────────────────────────────────────────────────────────────
i2c = board.I2C()
pca = PCA9685(i2c)
pca.frequency = 50            # 50 Hz is typical for hobby servos

# ── Constants ────────────────────────────────────────────────────────────
SERVO_CHANNEL  = 0
MIN_PULSE_US   = 500          # 0 °  ≈ 0.5 ms pulse
MAX_PULSE_US   = 2500         # 180 ° ≈ 2.5 ms pulse
PWM_PERIOD_US  = 1_000_000 / 50   # 20 000 µs period at 50 Hz

# ── Helpers ──────────────────────────────────────────────────────────────
def pulse_width_to_duty(pulse_us: float) -> int:
    """Convert a pulse width in µs to a 16-bit duty-cycle value."""
    duty_cycle = int((pulse_us / PWM_PERIOD_US) * 0xFFFF)
    return max(0, min(duty_cycle, 0xFFFF))   # clamp to [0, 65535]

def set_servo_angle(channel: int, angle: float) -> None:
    """Move the specified servo channel to the requested angle (0-180 °)."""
    pulse = MIN_PULSE_US + (angle / 180.0) * (MAX_PULSE_US - MIN_PULSE_US)
    pca.channels[channel].duty_cycle = pulse_width_to_duty(pulse)

# ── Main sweep loop ──────────────────────────────────────────────────────
try:
    STEP        = 10          # degrees per move
    DWELL_TIME  = 0.15        # seconds to wait at each step  (was 0.30)

    while True:
        # 0° → 180°
        for angle in range(0, 181, STEP):
            print(f"Setting angle → {angle:>3}°")
            set_servo_angle(SERVO_CHANNEL, angle)
            time.sleep(DWELL_TIME)

        # 180° → 0°
        for angle in range(180, -1, -STEP):
            print(f"Setting angle → {angle:>3}°")
            set_servo_angle(SERVO_CHANNEL, angle)
            time.sleep(DWELL_TIME)

except KeyboardInterrupt:
    print("\nStopping servo sweep.")
    pca.channels[SERVO_CHANNEL].duty_cycle = 0

