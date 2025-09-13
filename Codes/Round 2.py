import time
import threading
import collections
import logging
import numpy as np
import busio
import board
import smbus2
import cv2

try:
    import smbus
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False

try:
    import RPi.GPIO as GPIO
except Exception:
    from gpiozero import PWMLED
    import RPi.GPIO as GPIO  # fallback for PWM

try:
    from picamera2 import Picamera2
    PICAMERA_AVAILABLE = True
except Exception:
    PICAMERA_AVAILABLE = False

from gpiozero import PWMLED
import adafruit_vl53l0x
import adafruit_tcs34725


class Config:
    IN1 = 9
    IN2 = 11
    ENA = 10
    SERVO_PIN = 25
    C_PIN = 22
    NC_PIN = 24
    BUTTON_LED_PIN = 27
    TCS_LED_PIN = 17

    SENSOR_CHANNELS = {
        'FR': 1, 'FL': 2, 'RR': 3, 'RL': 4, 'F': 5, 'TCS': 0,
    }

    PWM_MOTOR_FREQ = 1000
    PWM_SERVO_FREQ = 50
    BUDGET_FRONT_US = 20000
    BUDGET_REAR_US = 33000
    FRONT_THRESHOLD_CM = 40.0
    SIDE_THRESHOLD_CM = 25.0
    STEER_MIN_DEG = 30
    STEER_MAX_DEG = 140
    COLOR_COUNT_FOR_LAP = 4
    TOTAL_LAPS = 3


# Logging
logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)


class TCA9548A:
    def __init__(self, i2c, address=0x70):
        self.i2c = i2c
        self.address = address
        self.disable_all()

    def select_channel(self, channel: int):
        if not (0 <= channel <= 7):
            raise ValueError("Channel must be 0–7")
        try:
            self.i2c.writeto(self.address, bytes([1 << channel]))
            return
        except Exception as e1:
            try:
                bus = smbus2.SMBus(1)
                bus.write_byte(self.address, 1 << channel)
                bus.close()
                return
            except Exception as e2:
                if SMBUS_AVAILABLE:
                    try:
                        bus = smbus.SMBus(1)
                        bus.write_byte(self.address, 1 << channel)
                        bus.close()
                        return
                    except Exception as e3:
                        raise RuntimeError(
                            f"TCA9548A channel select failed: {e1}, {e2}, {e3}"
                        )
                else:
                    raise RuntimeError(
                        f"TCA9548A channel select failed: {e1}, {e2}"
                    )

    def disable_all(self):
        try:
            self.i2c.writeto(self.address, bytes([0x00]))
            return
        except Exception as e1:
            try:
                bus = smbus2.SMBus(1)
                bus.write_byte(self.address, 0x00)
                bus.close()
                return
            except Exception as e2:
                if SMBUS_AVAILABLE:
                    try:
                        bus = smbus.SMBus(1)
                        bus.write_byte(self.address, 0x00)
                        bus.close()
                        return
                    except Exception as e3:
                        raise RuntimeError(
                            f"TCA9548A disable_all failed: {e1}, {e2}, {e3}"
                        )
                else:
                    raise RuntimeError(
                        f"TCA9548A disable_all failed: {e1}, {e2}"
                    )


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


class CameraSensor:
    def __init__(self, resolution=(320, 240), framerate=20):
        self.cv2 = cv2
        self.resolution = resolution
        self.framerate = framerate
        self.picamera = None
        if PICAMERA_AVAILABLE:
            self.picamera = Picamera2()
            cfg = self.picamera.create_preview_configuration(main={"format": "XRGB8888", "size": resolution})
            self.picamera.configure(cfg)
            self.picamera.start()
        else:
            self.cap = cv2.VideoCapture(0)

    def capture_frame(self):
        if self.picamera:
            arr = self.picamera.capture_array(wait=True)
            frame = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
        else:
            ret, frame = self.cap.read()
            if not ret:
                return None
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Flip upright
        frame = cv2.flip(frame, 0)
        return frame

    def detect_blocks_sections(self):
        frame = self.capture_frame()
        if frame is None:
            return None

        height, width, _ = frame.shape
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
            # Work in RGB already
            hsv = cv2.cvtColor(sec_img, cv2.COLOR_RGB2HSV)
            green_blocks = self._detect_rects(hsv, np.array([35, 50, 50]), np.array([90, 255, 255]))
            red_blocks = self._detect_rects(hsv, np.array([0, 70, 40]), np.array([10, 255, 255]))
            red_blocks += self._detect_rects(hsv, np.array([160, 70, 40]), np.array([180, 255, 255]))
            detected[sec_name] = {'green': green_blocks, 'red': red_blocks}

        return detected

    def _detect_rects(self, hsv, lo, hi):
        mask = cv2.inRange(hsv, lo, hi)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        found = []
        for cnt in contours:
            if cv2.contourArea(cnt) < 400:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if min(w, h) / max(w, h) < 0.4:
                continue
            found.append((x, y, w, h))
        return found


class Robot:
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(Config.C_PIN, GPIO.OUT)
        GPIO.setup(Config.NC_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.output(Config.C_PIN, GPIO.HIGH)

        self.motor = MotorController(Config.IN1, Config.IN2, Config.ENA, Config.PWM_MOTOR_FREQ)
        self.servo = ServoController(Config.SERVO_PIN, Config.PWM_SERVO_FREQ)

        self.button_led = PWMLED(Config.BUTTON_LED_PIN, frequency=100)
        self.tcs_led = PWMLED(Config.TCS_LED_PIN, frequency=100)

        i2c = busio.I2C(board.SCL, board.SDA)
        self.mux = TCA9548A(i2c)
        self.mux.select_channel(Config.SENSOR_CHANNELS['TCS'])
        self.color_sensor = adafruit_tcs34725.TCS34725(i2c)
        self._color_filter = RollingFilter(3)

        self.sensors = SensorManager(self.mux)
        self.sensors.start()

        self.camera = CameraSensor()
        self.orientation = None
        self.lap_count = 0

        # 🚀 Start motor immediately
        self.motor.forward(40)
        self.servo.set_angle(90)

    def is_blue(self, r, g, b):
        return b > 100 and b > r + 20 and b > g + 20

    def is_orange(self, r, g, b):
        return r > 150 and 80 < g < 150 and b < 80

    def determine_approach_side(self):
        front_distance = self.sensors.range_cm('F')
        return 'left' if front_distance > 20 else 'right'

    def map_detected_layout_to_direction(self, detected_sections, approach_side):
        for sec in detected_sections:
            green_ct = len(detected_sections[sec]['green'])
            red_ct = len(detected_sections[sec]['red'])
            logger.info(f"{sec}: Green={green_ct}, Red={red_ct}")

        if approach_side == 'left':
            if len(detected_sections.get('left_middle', {}).get('green', [])) > 0:
                return 'forward'
            else:
                return 'turn_right'
        else:
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
                logger.info(f"Approach={approach_side}, Direction={direction}")
                self.act_on_direction(direction)
                time.sleep(0.1)

        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        finally:
            self.shutdown()

    def shutdown(self):
        self.motor.stop()
        self.sensors.stop()
        GPIO.cleanup()
        if self.camera.picamera:
            self.camera.picamera.close()


def main():
    robot = Robot()
    robot.main_loop()


if __name__ == "__main__":
    main()
