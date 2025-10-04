# pi_client.py
"""
Pi #1 — Speed Measurement Unit
---------------------------------
- Two ultrasonic sensors (A and B)
- Measures travel time of object between A -> B
- Calculates speed
- Captures image from camera
- Sends image + metadata to server (/process)
"""

import time
import requests
import io
import cv2
import threading
import RPi.GPIO as GPIO

# ===== CONFIG =====
TRIG_A = 23
ECHO_A = 12
TRIG_B = 24
ECHO_B = 16

DIST_BETWEEN_M = 0.50  # meters between sensors
THRESHOLD_M = 0.4      # distance threshold to trigger
SERVER_URL = "http://YOUR_LAPTOP_IP:5000/process"  # <-- replace with your server IP
DEVICE_ID = "pi_speed_01"
CAMERA_INDEX = 0
# ===================

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_A, GPIO.OUT)
GPIO.setup(ECHO_A, GPIO.IN)
GPIO.setup(TRIG_B, GPIO.OUT)
GPIO.setup(ECHO_B, GPIO.IN)

def measure_distance(trig, echo):
    """Return distance (m) using ultrasonic sensor."""
    GPIO.output(trig, False)
    time.sleep(0.00005)
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    start = time.time()
    while GPIO.input(echo) == 0:
        start = time.time()
    while GPIO.input(echo) == 1:
        stop = time.time()
    return (stop - start) * 343 / 2  # m

def capture_image():
    """Capture a single frame from camera as JPEG bytes."""
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

def send_to_server(img_bytes, t1, t2, speed):
    """Send captured data to server."""
    files = {"image": ("img.jpg", io.BytesIO(img_bytes), "image/jpeg")}
    data = {"device_id": DEVICE_ID, "t1": str(t1), "t2": str(t2), "speed_m_s": str(speed)}
    try:
        r = requests.post(SERVER_URL, files=files, data=data, timeout=10)
        print("Server:", r.status_code, r.text[:200])
    except Exception as e:
        print("Send error:", e)

def main():
    state = "WAIT_A"
    t1 = None
    print("Speed sensor started. Waiting for object...")
    while True:
        try:
            dA = measure_distance(TRIG_A, ECHO_A)
            dB = measure_distance(TRIG_B, ECHO_B)
        except Exception as e:
            print("Sensor read error:", e)
            continue

        now = time.time()
        if state == "WAIT_A" and dA < THRESHOLD_M:
            t1 = now
            state = "WAIT_B"
            print("Object passed sensor A")
        elif state == "WAIT_B" and dB < THRESHOLD_M:
            t2 = now
            dt = t2 - t1 if t1 else 0
            if dt > 0:
                speed = DIST_BETWEEN_M / dt
                print(f"Speed = {speed:.2f} m/s")
                threading.Thread(target=lambda: send_to_server(capture_image(), t1, t2, speed), daemon=True).start()
            state = "WAIT_A"
        time.sleep(0.05)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Exiting...")
