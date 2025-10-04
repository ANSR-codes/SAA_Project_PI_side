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
SERVER_URL = "http://192.168.137.66:5000/check_actuator"  # <-- replace with your server IP
DEVICE_ID = "pi_actuator_01"
CAMERA_INDEX = 0

COOLDOWN_SECONDS = 5      # <---- new: cooldown after each trigger
MIN_TIME_BETWEEN_MEASURE = 0.03  # loop sleep
# ===================

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(ACTUATOR_PIN, GPIO.OUT)
GPIO.output(ACTUATOR_PIN, False)


def measure_distance(timeout=0.02):
    """Return distance (m) using ultrasonic sensor. Returns None on timeout."""
    GPIO.output(TRIG, False)
    time.sleep(0.00005)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start_time = time.time()
    # wait for echo to go high
    while GPIO.input(ECHO) == 0:
        if time.time() - start_time > timeout:
            return None
    pulse_start = time.time()
    # wait for echo to go low
    while GPIO.input(ECHO) == 1:
        if time.time() - pulse_start > timeout:
            return None
    pulse_end = time.time()
    elapsed = pulse_end - pulse_start
    return (elapsed * 343.0) / 2.0


def capture_image():
    """Capture one frame as JPEG bytes."""
    cam = cv2.VideoCapture(CAMERA_INDEX)
    if not cam.isOpened():
        raise RuntimeError("Camera not available")
    # warm up frames
    for _ in range(3):
        cam.read()
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
            print("Server error:", r.status_code, r.text[:200])
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
    print("Actuator controller running with cooldown:", COOLDOWN_SECONDS, "seconds")
    last_trigger = 0.0
    object_present = False

    try:
        while True:
            d = measure_distance()
            now = time.time()
            if d is None:
                # sensor timeout - treat as no object
                object_present = False
            else:
                # detect object
                if d < THRESHOLD_M:
                    # object is present
                    if not object_present:
                        # first detection edge (object just appeared)
                        object_present = True
                        # only handle if cooldown passed
                        if now - last_trigger >= COOLDOWN_SECONDS:
                            print(f"Object detected at {d:.2f} m — handling (cooldown passed).")
                            # spawn worker to capture/send and possibly actuate
                            threading.Thread(target=lambda: send_to_server(capture_image(), now), daemon=True).start()
                            last_trigger = now
                        else:
                            print(f"Object detected but in cooldown ({now - last_trigger:.2f}s elapsed).")
                    else:
                        # object still present: do nothing (debounce)
                        pass
                else:
                    # no object present
                    if object_present:
                        # object just left
                        print("Object left sensor area.")
                    object_present = False

            time.sleep(MIN_TIME_BETWEEN_MEASURE)
    except KeyboardInterrupt:
        print("Stopping (KeyboardInterrupt).")
    finally:
        GPIO.output(ACTUATOR_PIN, False)
        GPIO.cleanup()


if __name__ == "__main__":
    main()
