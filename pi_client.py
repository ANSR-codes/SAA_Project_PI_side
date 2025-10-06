# improved_pi_client.py
"""
Pi #1 — Speed Measurement Unit (camera-only)
 - Always captures an image from the camera when triggered (no random folder selection).
 - Adds timeouts to ultrasonic reading (prevents infinite blocking)
 - Prints debug info each loop
 - Uses time.monotonic() for timing
 - Safer thread closures when sending image
 - Better camera error handling
"""

import time
import requests
import io
import cv2
import threading
import RPi.GPIO as GPIO
import os

# ===== CONFIG =====
TRIG_A = 23
ECHO_A = 12
TRIG_B = 24
ECHO_B = 16

DIST_BETWEEN_M = 0.50  # meters between sensors
THRESHOLD_M = 0.4      # distance threshold to trigger
SERVER_URL = "http://10.90.239.18:5000/process"  # <-- replace with your server IP
DEVICE_ID = "pi_speed_01"
CAMERA_INDEX = 0
MEASURE_TIMEOUT = 0.03  # seconds (safe for sensors up to ~4 m)
POLL_DELAY = 0.05
# ==============================================================================

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(TRIG_A, GPIO.OUT)
GPIO.setup(TRIG_B, GPIO.OUT)
GPIO.setup(ECHO_A, GPIO.IN)
GPIO.setup(ECHO_B, GPIO.IN)

# Ensure TRIGs are low
GPIO.output(TRIG_A, False)
GPIO.output(TRIG_B, False)
time.sleep(0.1)


def measure_distance(trig, echo, timeout=MEASURE_TIMEOUT):
    """
    Trigger ultrasonic and measure pulse width. Returns distance in meters,
    or None on timeout/error.
    """
    try:
        # Send trigger pulse
        GPIO.output(trig, False)
        time.sleep(0.0002)
        GPIO.output(trig, True)
        time.sleep(0.00001)  # 10us pulse
        GPIO.output(trig, False)

        start_wait = time.monotonic()
        # wait for echo to go HIGH
        while GPIO.input(echo) == 0:
            if time.monotonic() - start_wait > timeout:
                return None
        pulse_start = time.monotonic()

        # wait for echo to go LOW
        while GPIO.input(echo) == 1:
            if time.monotonic() - pulse_start > timeout:
                return None
        pulse_end = time.monotonic()

        duration = pulse_end - pulse_start
        # Speed of sound ≈ 343 m/s; distance = (duration * speed) / 2
        distance = duration * 343.0 / 2.0
        return distance
    except Exception as e:
        print("measure_distance exception:", e, flush=True)
        return None


def capture_image_camera():
    """Capture a single frame from camera as JPEG bytes (raises on failure)."""
    cam = cv2.VideoCapture(CAMERA_INDEX)
    if not cam.isOpened():
        raise RuntimeError("Camera not available")
    # warm up
    for _ in range(3):
        cam.read()
    ret, frame = cam.read()
    cam.release()
    if not ret:
        raise RuntimeError("Failed to capture frame")
    _, buf = cv2.imencode('.jpg', frame)
    return buf.tobytes()


def get_image_bytes_for_send():
    """
    Always use the camera to capture bytes for send.
    Returns bytes or None on permanent failure.
    """
    try:
        return capture_image_camera()
    except Exception as e:
        print("Camera capture failed:", e, flush=True)
        # no other fallback; return None to indicate failure
        return None


def send_to_server(img_bytes, t1, t2, speed):
    """Send captured data to server."""
    if img_bytes is None:
        print("No image bytes available to send", flush=True)
        return
    files = {"image": ("img.jpg", io.BytesIO(img_bytes), "image/jpeg")}
    data = {"device_id": DEVICE_ID, "t1": str(t1), "t2": str(t2), "speed_m_s": str(speed)}
    try:
        r = requests.post(SERVER_URL, files=files, data=data, timeout=10)
        print("Server:", r.status_code, r.text[:200], flush=True)
    except Exception as e:
        print("Send error:", e, flush=True)


def main():
    state = "WAIT_A"
    t1 = None
    print("Speed sensor started. Waiting for object...", flush=True)

    while True:
        try:
            dA = measure_distance(TRIG_A, ECHO_A)
            dB = measure_distance(TRIG_B, ECHO_B)

            # debug print
            print(f"Distances -> A: {dA if dA is not None else 'TO'} m, "
                  f"B: {dB if dB is not None else 'TO'} m, state={state}", flush=True)

            now = time.monotonic()

            if state == "WAIT_A" and (dA is not None and dA < THRESHOLD_M):
                t1 = now
                state = "WAIT_B"
                print("Object passed sensor A", flush=True)

            elif state == "WAIT_B" and (dB is not None and dB < THRESHOLD_M):
                t2 = now
                dt = (t2 - t1) if t1 else 0
                if dt > 0:
                    speed = DIST_BETWEEN_M / dt
                    print(f"Speed = {speed:.2f} m/s (dt={dt:.4f}s)", flush=True)

                    # spawn thread that captures image from camera and sends it.
                    # freeze t1,t2,speed into defaults to avoid closure mutation issues
                    def worker(t1_=t1, t2_=t2, speed_=speed):
                        img = get_image_bytes_for_send()
                        if img is None:
                            print("Worker: camera capture failed; aborting send", flush=True)
                            return
                        send_to_server(img, t1_, t2_, speed_)

                    threading.Thread(target=worker, daemon=True).start()

                state = "WAIT_A"

            time.sleep(POLL_DELAY)

        except KeyboardInterrupt:
            print("Exiting (KeyboardInterrupt)", flush=True)
            break
        except Exception as e:
            print("Main loop error:", e, flush=True)
            time.sleep(0.2)

    GPIO.cleanup()


if __name__ == "__main__":
    main()
