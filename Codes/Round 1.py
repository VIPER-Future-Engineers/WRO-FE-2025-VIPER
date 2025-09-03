import time
import board
import busio
import threading
import collections
import logging

import adafruit_vl53l0x
import adafruit_tcs34725
import RFRobot_I2C_Multiplexer as rf_mux
import RPi.GPIO as GPIO

# ----------------------------------------------------------------------------- 
# CONFIGURATION
# -----------------------------------------------------------------------------
class Config:
    IN1            = 9
    IN2            = 11
    ENA            = 10
    SERVO_PIN      = 25
    BUTTON_PIN     = 23   # Not used for stop
    BUTTON_LED_PIN = 27
    TCS_LED_PIN    = 17

    PWM_MOTOR_FREQ      = 1000
    PWM_SERVO_FREQ      = 50
    PWM_BUTTON_LED_FREQ = 500
    PWM_TCS_LED_FREQ    = 500

    SENSOR_CHANNELS = {
        'FR': 1,
        'FL': 2,
        'F':  3,
        'RL': 4,
        'RR': 5,
        'TCS': 0,
    }

    BUDGET_FRONT_US = 20000
    BUDGET_REAR_US  = 33000

    FRONT_THRESHOLD_CM = 40.0

    # Fully adjustable steering
    STEER_MIN_DEG = 30
    STEER_MAX_DEG = 150

    COLOR_COUNT_FOR_LAP = 4
    TOTAL_LAPS          = 3

# ----------------------------------------------------------------------------- 
# LOGGING
# -----------------------------------------------------------------------------
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)

# ----------------------------------------------------------------------------- 
# HELPER CLASSES
# -----------------------------------------------------------------------------
class RollingFilter:
    def __init__(self, size: int = 3):
        self._buf = collections.deque(maxlen=size)

    def add(self, sample: float) -> float:
        self._buf.append(sample)
        return sum(self._buf) / len(self._buf)

class MotorController:
    def __init__(self, in1, in2, pwm_pin, freq):
        GPIO.setup(in1, GPIO.OUT)
        GPIO.setup(in2, GPIO.OUT)
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
        duty = 2 + (angle / 18)
        self._pwm.ChangeDutyCycle(duty)
        self._last_angle = angle

class LEDController:
    def __init__(self, pin, freq):
        GPIO.setup(pin, GPIO.OUT)
        self._pwm = GPIO.PWM(pin, freq)
        self._pwm.start(0)

    def set_duty(self, duty):
        duty = max(0, min(100, duty))
        self._pwm.ChangeDutyCycle(duty)

    def stop(self):
        self._pwm.ChangeDutyCycle(0)

# ----------------------------------------------------------------------------- 
# SENSOR MANAGER (Threaded)
# -----------------------------------------------------------------------------
class SensorManager(threading.Thread):
    def __init__(self, mux: rf_mux.I2C_Multiplexer):
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
            mux.channel_enable(ch)
            sens = adafruit_vl53l0x.VL53L0X(mux.i2c)
            sens.measurement_timing_budget = (
                Config.BUDGET_FRONT_US if name.startswith('F') else Config.BUDGET_REAR_US
            )
            sens.start_continuous()
            self._sensors[name] = sens
            self._filters[name] = RollingFilter(3)
            self._latest[name] = 0

    def run(self):
        while self._running:
            for name, sensor in self._sensors.items():
                mm = sensor.range
                avg_mm = self._filters[name].add(mm)
                with self._lock:
                    self._latest[name] = avg_mm / 10.0
            time.sleep(0.03)

    def range_cm(self, name):
        with self._lock:
            return self._latest.get(name, 0.0)

    def stop(self):
        self._running = False

# ----------------------------------------------------------------------------- 
# HEARTBEAT THREAD
# -----------------------------------------------------------------------------
class HeartbeatThread(threading.Thread):
    def __init__(self, button_led: LEDController, tcs_led: LEDController):
        super().__init__(daemon=True)
        self.button_led = button_led
        self.tcs_led = tcs_led
        self._running = True

    def run(self):
        while self._running:
            # Simple continuous pulse
            for duty in [100, 30, 80, 20, 0]:
                self.button_led.set_duty(duty)
                time.sleep(0.1)
            # TCS LED flicker
            for duty in [40, 100, 30, 90]:
                self.tcs_led.set_duty(duty)
                time.sleep(0.12)
            for duty in range(70, -1, -10):
                self.tcs_led.set_duty(duty)
                time.sleep(0.15)

    def stop(self):
        self._running = False

# ----------------------------------------------------------------------------- 
# ROBOT CLASS
# -----------------------------------------------------------------------------
class Robot:
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        self.motor      = MotorController(Config.IN1, Config.IN2, Config.ENA, Config.PWM_MOTOR_FREQ)
        self.servo      = ServoController(Config.SERVO_PIN, Config.PWM_SERVO_FREQ,
                                          min_deg=Config.STEER_MIN_DEG,
                                          max_deg=Config.STEER_MAX_DEG)
        self.button_led = LEDController(Config.BUTTON_LED_PIN, Config.PWM_BUTTON_LED_FREQ)
        self.tcs_led    = LEDController(Config.TCS_LED_PIN, Config.PWM_TCS_LED_FREQ)

        i2c = busio.I2C(board.SCL, board.SDA)
        global mux
        mux = rf_mux.I2C_Multiplexer(i2c, address=0x70)

        self.sensors      = SensorManager(mux)
        self.sensors.start()
        self.color_sensor = adafruit_tcs34725.TCS34725(mux.i2c)
        self._color_filter = RollingFilter(3)

        self.orientation = None
        self.lap_count = 0
        self.color_seen = 0

        self.heartbeat = HeartbeatThread(self.button_led, self.tcs_led)
        self.heartbeat.start()

    def detect_orientation(self):
        while self.orientation is None:
            r, g, b = self.color_sensor.color_rgb_bytes
            r_avg = self._color_filter.add(r)
            g_avg = self._color_filter.add(g)
            b_avg = self._color_filter.add(b)
            if b_avg > r_avg and b_avg > g_avg and b_avg > 50:
                self.orientation = 'CCW'
            elif r_avg > g_avg and r_avg > b_avg and r_avg > 50:
                self.orientation = 'CW'
            else:
                time.sleep(0.05)
        logger.info("Orientation set to %s", self.orientation)

    def check_lap(self):
        r, g, b = self.color_sensor.color_rgb_bytes
        r_avg = self._color_filter.add(r)
        g_avg = self._color_filter.add(g)
        b_avg = self._color_filter.add(b)
        match = ((self.orientation == 'CCW' and b_avg > r_avg and b_avg > g_avg) or
                 (self.orientation == 'CW'  and r_avg > g_avg and r_avg > b_avg))
        self.color_seen = self.color_seen + 1 if match else 0
        if self.color_seen >= Config.COLOR_COUNT_FOR_LAP:
            self.lap_count += 1
            self.color_seen = 0
            logger.info("🏁 Lap %d completed!", self.lap_count)

    def main_loop(self):
        try:
            while self.orientation is None:
                self.detect_orientation()
            while self.lap_count < Config.TOTAL_LAPS:
                self.check_lap()

                front = self.sensors.range_cm('F')
                fl = self.sensors.range_cm('FL')
                fr = self.sensors.range_cm('FR')

                if front < Config.FRONT_THRESHOLD_CM:
                    diff = fr - fl
                    steer = 90 + diff
                else:
                    rl = self.sensors.range_cm('RL')
                    rr = self.sensors.range_cm('RR')
                    diff = rr - rl if self.orientation == 'CCW' else rl - rr
                    steer = 90 + diff

                steer = max(Config.STEER_MIN_DEG, min(Config.STEER_MAX_DEG, steer))
                self.servo.set_angle(steer)
                self.motor.forward(50)
                time.sleep(0.03)

        except Exception:
            logger.exception("Unhandled exception in main_loop")
        finally:
            self.shutdown()

    def shutdown(self):
        logger.info("Shutdown initiated.")
        self.motor.stop()
        self.heartbeat.stop()
        self.button_led.stop()
        self.tcs_led.stop()
        self.sensors.stop()
        GPIO.cleanup()

# ----------------------------------------------------------------------------- 
# MAIN
# -----------------------------------------------------------------------------
def main():
    robot = Robot()
    robot.main_loop()

if __name__ == "__main__":
    main()
