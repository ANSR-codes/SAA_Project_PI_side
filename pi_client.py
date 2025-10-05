# test_ultrasonic.py
import time
import RPi.GPIO as GPIO

# CONFIG (BCM)
TRIG = 23
ECHO = 12
THRESHOLD_M = 0.4       # object considered "present" if distance < threshold
MEASURE_INTERVAL = 0.2  # seconds between measurements
ECHO_TIMEOUT = 0.02     # seconds (20 ms) max wait for echo

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.output(TRIG, False)

def measure_distance(trig=TRIG, echo=ECHO, timeout=ECHO_TIMEOUT):
    """
    Return distance in meters or None on timeout/error.
    Uses non-blocking timeouts to avoid hanging.
    """
    # ensure trigger low
    GPIO.output(trig, False)
    time.sleep(0.00005)

    # send 10us pulse
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start_time = time.time()
    # wait for echo to go HIGH
    while GPIO.input(echo) == 0:
        if time.time() - start_time > timeout:
            return None
    pulse_start = time.time()

    # wait for echo to go LOW
    while GPIO.input(echo) == 1:
        if time.time() - pulse_start > timeout:
            return None
    pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    # speed of sound 343 m/s, distance = time * speed / 2
    distance_m = (pulse_duration * 343.0) / 2.0
    return distance_m

def main():
    print("Ultrasonic test (TRIG=23, ECHO=12). Press Ctrl+C to stop.")
    try:
        while True:
            d = measure_distance()
            if d is None:
                print("No echo (timeout) — no object detected or wiring/sensor problem.")
            else:
                print(f"Distance: {d:.3f} m", end='')
                if d < THRESHOLD_M:
                    print("  <-- OBJECT DETECTED")
                else:
                    print("  (no object)")
            time.sleep(MEASURE_INTERVAL)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
