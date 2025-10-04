# actuator_pi.py
"""
Pi #2 — Actuator Controller Unit
---------------------------------
- One ultrasonic sensor (detects object)
- Captures image
- Sends to server (/check_actuator)
- Moves actuator if server says "open"
"""

import time
import io
import requests
import cv2
import threading
import RPi.GPIO as GPIO

# ===== CONFIG =====
TRIG = 24
ECHO = 16
ACTUATOR_PIN = 22  # relay or motor control GPIO
THRESHOLD_M = 0.4
SERVER_URL = "http://YOUR_LAPTOP_IP:5000/check_actuator"  # <-- replace with your server IP
DEVICE_ID = "pi_actuator_01"
CAMERA_INDEX = 0
# ===================

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(ACTUATOR_PIN, GPIO.OUT)
GPIO.output(ACTUATOR_PIN, False)

def measure_distance():
    """Return distance (m) using ultrasonic sensor."""
    GPIO.output(TRIG, False)
    time.sleep(0.00005)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    start = time.time()
    while GPIO.input(ECHO) == 0:
        start = time.time()
    while GPIO.input(ECHO) == 1:
        stop = time.time()
    return (stop - start) * 343 / 2

def capture_image():
    """Capture one frame as JPEG bytes."""
    cam = cv2.VideoCapture(CAMERA_INDEX)
    if not cam.isOpened():
        raise RuntimeError("Camera not available")
    for _ in range(3): cam.read()
    ret, frame = cam.read()
    cam.release()
    if not ret:
        raise RuntimeError("Failed to capture frame")
    _, buf = cv2.imencode('.jpg', frame)
    return buf.tobytes()

def actuator_pulse(duration=0.5):
    """Pulse actuator (relay HIGH for duration seconds)."""
    GPIO.output(ACTUATOR_PIN, True)
    time.sleep(duration)
    GPIO.output(ACTUATOR_PIN, False)

def send_to_server(img_bytes, t_detect):
    """Send image to server and handle response."""
    files = {"image": ("img.jpg", io.BytesIO(img_bytes), "image/jpeg")}
    data = {"device_id": DEVICE_ID, "t": str(t_detect)}
    try:
        r = requests.post(SERVER_URL, files=files, data=data, timeout=10)
        if r.status_code != 200:
            print("Server error:", r.status_code)
            return
        resp = r.json()
        print("Server response:", resp)
        if resp.get("action") == "open":
            print("Actuator: OPEN command received")
            actuator_pulse()
        else:
            print("No actuation needed.")
    except Exception as e:
        print("Error contacting server:", e)

def main():
    print("Actuator controller running...")
    last_trigger = 0
    while True:
        try:
            d = measure_distance()
            if d < THRESHOLD_M and time.time() - last_trigger > 1:
                print(f"Object detected at {d:.2f} m")
                threading.Thread(target=lambda: send_to_server(capture_image(), time.time()), daemon=True).start()
                last_trigger = time.time()
        except Exception as e:
            print("Sensor error:", e)
        time.sleep(0.05)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Stopped.")
