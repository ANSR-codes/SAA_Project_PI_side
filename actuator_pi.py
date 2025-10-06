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
TRIG_PIN = 24
ECHO_PIN = 16
ACTUATOR_PIN = 22  # relay or motor control GPIO
THRESHOLD_M = 0.4
SERVER_URL = "http://192.168.137.66:5000/check_actuator"  # <-- replace with your server IP
DEVICE_ID = "pi_actuator_01"
CAMERA_INDEX = 0

COOLDOWN_SECONDS = 5      # <---- new: cooldown after each trigger
MIN_LOOP_DELAY = 0.03     # main loop sleep

# Servo angles (customize)
SERVO_OPEN_ANGLE = 90     # angle to move to when "open"
SERVO_CLOSED_ANGLE = 0    # resting/closed angle
SERVO_MOVE_DELAY = 1   # seconds to hold position (servo travel time)
SERVO_PWM_FREQ = 50       # Hz for hobby servos

# ===== end config =====

GPIO.setmode(GPIO.BCM)

# Setup pins
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(ACTUATOR_PIN, GPIO.OUT)

# Initialize servo PWM
servo = GPIO.PWM(ACTUATOR_PIN, SERVO_PWM_FREQ)
servo.start(0)  # start with 0% duty to be safe


def measure_distance(timeout=0.02):
    """Measure distance (m) from HC-SR04 style sensor. Returns None on timeout."""
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.00005)
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    start_t = time.time()
    # wait for echo high
    while GPIO.input(ECHO_PIN) == 0:
        if time.time() - start_t > timeout:
            return None
    pulse_start = time.time()
    # wait for echo low
    while GPIO.input(ECHO_PIN) == 1:
        if time.time() - pulse_start > timeout:
            return None
    pulse_end = time.time()
    elapsed = pulse_end - pulse_start
    # speed of sound ~343 m/s -> distance = elapsed * 343 / 2
    return (elapsed * 343.0) / 2.0


def capture_image():
    """Capture a single frame from camera and return JPEG bytes."""
    cam = cv2.VideoCapture(CAMERA_INDEX)
    if not cam.isOpened():
        raise RuntimeError("Camera not available (index={})".format(CAMERA_INDEX))
    # warm up
    for _ in range(3):
        cam.read()
    ret, frame = cam.read()
    cam.release()
    if not ret:
        raise RuntimeError("Failed to capture frame")
    _, buf = cv2.imencode('.jpg', frame)
    return buf.tobytes()


def servo_set_angle(angle):
    """
    Set hobby servo to angle (0..180).
    Duty cycle mapping: ~2% -> 0deg, ~12% -> 180deg (may vary by servo).
    We set pulse then briefly wait and set duty to 0 to reduce jitter.
    """
    if angle < 0: angle = 0
    if angle > 180: angle = 180
    duty = 2.0 + (angle / 18.0)   # typical mapping
    try:
        servo.ChangeDutyCycle(duty)
        time.sleep(SERVO_MOVE_DELAY)
    finally:
        # Stop sending pulses to reduce jitter; leaving servo holding may be necessary for your use-case
        servo.ChangeDutyCycle(0)


def actuator_open():
    """Move servo to 'open' position."""
    servo_set_angle(SERVO_OPEN_ANGLE)


def actuator_close():
    """Move servo to 'closed' position."""
    servo_set_angle(SERVO_CLOSED_ANGLE)


# add/update at top of file or near config
PASS_DURATION = float(os.environ.get("PASS_DURATION", 6.0))  # seconds to open when action == "open"

def send_to_server_and_act(img_bytes, t_detect):
    """Send image to server and perform action based on response.action"""
    files = {"image": ("img.jpg", io.BytesIO(img_bytes), "image/jpeg")}
    data = {"device_id": DEVICE_ID, "t": str(t_detect)}
    try:
        r = requests.post(SERVER_URL, files=files, data=data, timeout=10)
        if r.status_code != 200:
            print("Server returned status", r.status_code, r.text[:200])
            return
        resp = r.json()
        print("Server response:", resp)
        action = resp.get("action")
        if action == "open":
            # non-flagged: open briefly to let object pass, then close
            print("Actuator command: OPEN -> moving servo to open for PASS_DURATION")
            actuator_open()
            time.sleep(PASS_DURATION)
            actuator_close()
            print("Actuator: closed after PASS_DURATION")
        elif action == "close":
            # flagged: keep gate closed and mark it flagged (stay closed)
            print("Actuator command: CLOSE -> keeping gate closed (flagged)")
            actuator_close()
            # do NOT auto-open; wait for future server instruction to open
        else:
            print("No actuation (action missing or 'none')")
    except Exception as e:
        print("Error contacting server or acting:", e)

def main_loop():
    print("Actuator (servo) controller running. Cooldown:", COOLDOWN_SECONDS, "s")
    last_handled = 0.0
    object_present = False

    try:
        while True:
            d = measure_distance()
            now = time.time()

            if d is None:
                # sensor timeout => treat as no object
                object_present = False
            else:
                if d < THRESHOLD_M:
                    # object present
                    if not object_present:
                        # rising edge: object just appeared
                        object_present = True
                        if now - last_handled >= COOLDOWN_SECONDS:
                            print(f"Object detected at {d:.2f} m; handling (cooldown ok)")
                            # spawn worker to avoid blocking main loop
                            threading.Thread(target=lambda: send_to_server_and_act(capture_image(), now), daemon=True).start()
                            last_handled = now
                        else:
                            print(f"Object detected but still in cooldown ({now - last_handled:.2f}s elapsed)")
                    else:
                        # still present -> do nothing (debounce)
                        pass
                else:
                    # no object present
                    if object_present:
                        print("Object left sensor area")
                    object_present = False

            time.sleep(MIN_LOOP_DELAY)
    except KeyboardInterrupt:
        print("KeyboardInterrupt -> exiting")
    finally:
        # cleanup
        try:
            servo.stop()
        except Exception:
            pass
        GPIO.output(ACTUATOR_PIN, False)
        GPIO.cleanup()


if __name__ == "__main__":
    main_loop()
