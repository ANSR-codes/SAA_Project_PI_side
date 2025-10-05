# servo_test.py
"""
Quick test for SG90 micro servo on Raspberry Pi.
Moves servo to 0°, 90°, and 180° repeatedly every 2 seconds.

⚠️ Important:
- Connect servo signal wire to GPIO 22 (or change SERVO_PIN below).
- Power servo from 5V (not 3.3V!) and use a common ground with the Pi.
"""

import RPi.GPIO as GPIO
import time

# === CONFIG ===
SERVO_PIN = 22       # GPIO pin connected to servo signal (orange wire)
PWM_FREQ = 50        # Hz (20 ms period typical for servos)
MOVE_DELAY = 0.5     # seconds to hold position
# ===============

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

servo = GPIO.PWM(SERVO_PIN, PWM_FREQ)
servo.start(0)  # 0% duty = off

def set_angle(angle):
    """Move servo to angle (0–180°)."""
    if angle < 0: angle = 0
    if angle > 180: angle = 180
    duty = 2 + (angle / 18)  # maps 0°->2%, 180°->12%
    print(f"→ Moving to {angle}°, duty={duty:.2f}%")
    servo.ChangeDutyCycle(duty)
    time.sleep(MOVE_DELAY)
    servo.ChangeDutyCycle(0)  # stop signal to reduce jitter

try:
    print("Starting SG90 servo test. Press Ctrl+C to stop.")
    while True:
        for angle in [0, 90, 180, 90, 0]:
            set_angle(angle)
            time.sleep(1.5)
except KeyboardInterrupt:
    print("\nStopping test.")
finally:
    servo.stop()
    GPIO.cleanup()
    print("GPIO cleaned up.")
