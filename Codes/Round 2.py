#!/usr/bin/env python3
import time, threading, collections, logging
import numpy as np
import sys
import os
import busio
import board
import smbus2
import cv2

try:
    import RPi.GPIO as GPIO
except Exception:
    from gpiozero import LED, PWMLED, Button
    import RPi.GPIO as GPIO  # fallback for PWM


try:
    from picamera2 import Picamera2
    PICAMERA_AVAILABLE = True
except Exception:
    PICAMERA_AVAILABLE = False

# ----------------------------------------------------------------------------- 
# CONFIGURATION
# ----------------------------------------------------------------------------- 
class Config:
    IN1            = 9
    IN2            = 11
    ENA            = 10
    SERVO_PIN      = 25
    BUTTON_PIN     = 23
    C_PIN          = 22
    NC_PIN         = 24
    BUTTON_LED_PIN = 27
    TCS_LED_PIN    = 17
    PWM_MOTOR_FREQ      = 1000
    PWM_SERVO_FREQ      = 50
    SENSOR_CHANNELS = {
        'FR': 1,
        'FL': 2,
        'RR': 3,
        'RL': 4,
        'F':  5,
        'TCS': 0,
    }
    BUDGET_FRONT_US = 20000
    BUDGET_REAR_US  = 33000
    FRONT_THRESHOLD_CM = 40.0
    SIDE_THRESHOLD_CM  = 25.0
    STEER_MIN_DEG = 30
    STEER_MAX_DEG = 140
    COLOR_COUNT_FOR_LAP = 4
    TOTAL_LAPS = 3

# ----------------------------------------------------------------------------- 
# LOGGING
# ----------------------------------------------------------------------------- 
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)

# ----------------------------------------------------------------------------- 
# TCA9548A MULTIPLEXER (primary) + SMBus fallback
# ----------------------------------------------------------------------------- 
class TCA9548A:
    def __init__(self, i2c, address=0x70):
        self.i2c = i2c
        self.address = address
        self.disable_all()

    def select_channel(self, channel: int):
        if 0 <= channel <= 7:
            try:
                self.i2c.writeto(self.address, bytes([1 << channel]))
            except Exception:
                bus = smbus2.SMBus(1)
                bus.write_byte(self.address, 1 << channel)
        else:
            raise ValueError("Channel must be 0–7")

    def disable_all(self):
        try:
            self.i2c.writeto(self.address, bytes([0x00]))
        except Exception:
            bus = smbus2.SMBus(1)
            bus.write_byte(self.address, 0x00)

# ----------------------------------------------------------------------------- 
# HELPERS
# ----------------------------------------------------------------------------- 
class RollingFilter:
    def __init__(self, size=3):
        self._buf = collections.deque(maxlen=size)
    def add(self, sample):
        self._buf.append(sample)
        return float(np.mean(self._buf))

class MotorController:
    def __init__(self, in1, in2, pwm_pin, freq):
        GPIO.setup(in1, GPIO.OUT)
        GPIO.setup(in2, GPIO.OUT)
        GPIO.setup(pwm_pin, GPIO.OUT)
        self._in1 = in1
        self._in2 = in2
        self._pwm = GPIO.PWM(pwm_pin, freq)
        self._pwm.start(0)
        self._last_speed = None
    def forward(self, speed):
        if self._last_speed != speed:
            GPIO.output(self._in1, GPIO.LOW)
            GPIO.output(self._in2, GPIO.HIGH)
            self._pwm.ChangeDutyCycle(speed)
            self._last_speed = speed
    def stop(self):
        GPIO.output(self._in1, GPIO.HIGH)
        GPIO.output(self._in2, GPIO.HIGH)
        self._pwm.ChangeDutyCycle(0)
        self._last_speed = 0

class ServoController:
    def __init__(self, pin, freq, min_deg=0, max_deg=180):
        GPIO.setup(pin, GPIO.OUT)
        self._pwm = GPIO.PWM(pin, freq)
        self._pwm.start(7.5)
        self._last_angle = None
        self._min = min_deg
        self._max = max_deg

    def set_angle(self, angle):
        angle = max(self._min, min(self._max, angle))
        if self._last_angle == angle:
            return
        duty = 2 + (angle / 16)
        self._pwm.ChangeDutyCycle(duty)
        self._last_angle = angle

class LEDAnimation:
    """Heartbeat LED animation."""
    def __init__(self, button_led, tcs_led):
        self.button_led = button_led
        self.tcs_led = tcs_led
        self._running = True
        self._thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
    def start(self):
        self._running = True
        self._thread.start()
    def stop(self):
        self._running = False
        self.button_led.value = 0
        self.tcs_led.value = 0
    def _heartbeat_loop(self):
        while self._running:
            for duty in [0.0,0.2,0.5,0.8,1.0,0.6,0.3,0.0]:
                self.button_led.value = duty
                time.sleep(0.1)
            time.sleep(0.3)

# ----------------------------------------------------------------------------- 
# SENSOR MANAGER
# ----------------------------------------------------------------------------- 
class SensorManager(threading.Thread):
    def __init__(self, mux):
        super().__init__(daemon=True)
        self._sensors = {}
        self._filters = {}
        self._mux = mux
        self._latest = {}
        self._lock = threading.Lock()
        self._running = True
        for name, ch in Config.SENSOR_CHANNELS.items():
            if name == 'TCS':
                continue
            try:
                self._mux.select_channel(ch)
                time.sleep(0.02)
                import adafruit_vl53l0x
                sens = adafruit_vl53l0x.VL53L0X(self._mux.i2c)
                sens.measurement_timing_budget = (
                    Config.BUDGET_FRONT_US if name.startswith('F') else Config.BUDGET_REAR_US
                )
                sens.start_continuous()
                self._sensors[name] = sens
                self._filters[name] = RollingFilter(3)
                self._latest[name] = 0
                logger.info("VL53L0X sensor %s initialized on channel %d", name, ch)
            except Exception as e:
                logger.error("Failed to init sensor %s on channel %d: %s", name, ch, e)
    def run(self):
        while self._running:
            for name, sensor in self._sensors.items():
                try:
                    mm = sensor.range
                    avg_mm = self._filters[name].add(mm)
                    with self._lock:
                        self._latest[name] = avg_mm / 10.0
                except Exception:
                    pass
            time.sleep(0.02)
    def range_cm(self, name):
        with self._lock:
            return self._latest.get(name, 0.0)
    def stop(self):
        self._running = False

# ----------------------------------------------------------------------------- 
# CameraSensor with section split block detection
# ----------------------------------------------------------------------------- 
class CameraSensor:
    def __init__(self, resolution=(320,240), framerate=20):
        self.cv2 = cv2
        self.resolution = resolution
        self.framerate = framerate
        self.picamera = None
        if PICAMERA_AVAILABLE:
            from picamera2 import Picamera2
            self.picamera = Picamera2()
            cfg = self.picamera.create_preview_configuration(
                main={"format":"XRGB8888","size":resolution}
            )
            self.picamera.configure(cfg)
            self.picamera.start()

    def capture_frame(self):
        if self.picamera:
            arr = self.picamera.capture_array(wait=True)
            frame = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
        else:
            cap = cv2.VideoCapture(0)
            ret, frame = cap.read()
            if not ret:
                return None
            cap.release()
        return frame

    def _detect_rects(self, hsv, lo, hi):
        mask = cv2.inRange(hsv, lo, hi)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        found = []
        for cnt in contours:
            if cv2.contourArea(cnt) < 400:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if min(w,h)/max(w,h) < 0.4:
                continue
            found.append((x,y,w,h))
        return found

    def detect_blocks_sections(self):
        frame = self.capture_frame()
        if frame is None:
            return None

        height, width, _ = frame.shape

        # Split camera image into 6 sections: left/right, top/middle/bottom
        sections = {
            'left_top': frame[:height // 3, :width // 2],
            'left_middle': frame[height // 3: 2 * height // 3, :width // 2],
            'left_bottom': frame[2 * height // 3:, :width // 2],
            'right_top': frame[:height // 3, width // 2:],
            'right_middle': frame[height // 3: 2 * height // 3, width // 2:],
            'right_bottom': frame[2 * height // 3:, width // 2:],
        }

        detected = {}
        for sec_name, sec_img in sections.items():
            hsv = cv2.cvtColor(sec_img, cv2.COLOR_BGR2HSV)
            green_blocks = self._detect_rects(hsv, np.array([35,50,50]), np.array([90,255,255]))
            red_blocks = self._detect_rects(hsv, np.array([0,70,40]), np.array([10,255,255]))
            red_blocks += self._detect_rects(hsv, np.array([160,70,40]), np.array([180,255,255]))
            detected[sec_name] = {'green': green_blocks, 'red': red_blocks}

        return detected

# ----------------------------------------------------------------------------- 
# ROBOT CLASS
# ----------------------------------------------------------------------------- 
class Robot:
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.motor = MotorController(Config.IN1, Config.IN2, Config.ENA, Config.PWM_MOTOR_FREQ)
        self.servo = ServoController(Config.SERVO_PIN, Config.PWM_SERVO_FREQ)
        self.button_led = PWMLED(Config.BUTTON_LED_PIN)
        self.tcs_led = PWMLED(Config.TCS_LED_PIN)
        i2c = busio.I2C(board.SCL, board.SDA)
        self.mux = TCA9548A(i2c)
        self.sensors = SensorManager(self.mux)
        self.sensors.start()
        self.camera = CameraSensor()
        self.led_anim = LEDAnimation(self.button_led, self.tcs_led)
        self.led_anim.start()
        self.orientation = None
        self.lap_count = 0
        GPIO.setup(Config.BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(Config.BUTTON_PIN, GPIO.FALLING, callback=self.button_callback, bouncetime=50)

    def button_callback(self, channel):
        logger.info("Button pressed")

    def determine_approach_side(self):
        # Placeholder approach side determination example based on front sensor distance
        front_distance = self.sensors.range_cm('F')
        return 'left' if front_distance > 20 else 'right'

    def map_detected_layout_to_direction(self, detected_sections, approach_side):
        """
        This is a stub example. Replace this logic with matching your actual
        layouts and mapping to directions depending on the approach_side.
        """
        # Debug logging block counts for each section
        for sec in detected_sections:
            green_ct = len(detected_sections[sec]['green'])
            red_ct = len(detected_sections[sec]['red'])
            logger.info(f"{sec}: Green blocks={green_ct}, Red blocks={red_ct}")

        # Example: decision map for demonstration
        if approach_side == 'left':
            # Sample rule: If any green block detected in left_middle, go forward else turn right
            if len(detected_sections.get('left_middle', {}).get('green', [])) > 0:
                return 'forward'
            else:
                return 'turn_right'
        else:
            # if any red block detected in right_middle, go forward else turn left
            if len(detected_sections.get('right_middle', {}).get('red', [])) > 0:
                return 'forward'
            else:
                return 'turn_left'

    def act_on_direction(self, direction):
        if direction == 'forward':
            self.motor.forward(80)
            self.servo.set_angle(90)
        elif direction == 'turn_left':
            self.motor.forward(60)
            self.servo.set_angle(Config.STEER_MIN_DEG)
        elif direction == 'turn_right':
            self.motor.forward(60)
            self.servo.set_angle(Config.STEER_MAX_DEG)
        else:
            self.motor.stop()

    def main_loop(self):
        try:
            while self.lap_count < Config.TOTAL_LAPS:
                detected_sections = self.camera.detect_blocks_sections()
                if detected_sections is None:
                    continue
                approach_side = self.determine_approach_side()
                direction = self.map_detected_layout_to_direction(detected_sections, approach_side)
                logger.info(f"Approach: {approach_side}, Direction: {direction}")
                self.act_on_direction(direction)
                time.sleep(0.1)
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        finally:
            self.shutdown()

    def shutdown(self):
        self.led_anim.stop()
        self.motor.stop()
        self.sensors.stop()
        GPIO.cleanup()
        if self.camera.picamera:
            self.camera.picamera.close()

# ----------------------------------------------------------------------------- 
# MAIN
# ----------------------------------------------------------------------------- 
def main():
    robot = Robot()
    robot.main_loop()

if __name__ == "__main__":
    main()
