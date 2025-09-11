import time
import threading
import collections
import logging
import multiprocessing

import board
import busio
import adafruit_vl53l0x
import adafruit_tcs34725
import RPi.GPIO as GPIO

# Data/processing libs
import pandas as pd
import numpy as np
import scipy
from gpiozero import PWMLED, Button

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
# TCA9548A MULTIPLEXER
# ----------------------------------------------------------------------------- 
class TCA9548A:
    def __init__(self, i2c, address=0x70):
        self.i2c = i2c
        self.address = address
        self.disable_all()

    def select_channel(self, channel: int):
        if 0 <= channel <= 7:
            self.i2c.writeto(self.address, bytes([1 << channel]))
        else:
            raise ValueError("Channel must be 0–7")

    def disable_all(self):
        self.i2c.writeto(self.address, bytes([0x00]))

# ----------------------------------------------------------------------------- 
# HELPERS
# ----------------------------------------------------------------------------- 
class RollingFilter:
    """Simple rolling average filter."""
    def __init__(self, size: int = 3):
        self._buf = collections.deque(maxlen=size)
    def add(self, sample: float) -> float:
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

# ----------------------------------------------------------------------------- 
# SENSOR MANAGER (Threaded)
# ----------------------------------------------------------------------------- 
class SensorManager(threading.Thread):
    def __init__(self, mux: TCA9548A):
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
            time.sleep(0.02)  # faster refresh than before

    def range_cm(self, name):
        with self._lock:
            return self._latest.get(name, 0.0)

    def stop(self):
        self._running = False

# ----------------------------------------------------------------------------- 
# LED ANIMATION (robot personality)
# ----------------------------------------------------------------------------- 
class LEDAnimation:
    def __init__(self, button_led: PWMLED, tcs_led: PWMLED):
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
        """Heartbeat = personality while waiting."""
        while self._running:
            for duty in [1.0, 0.3, 0.8, 0.2, 0.0]:
                self.button_led.value = duty
                time.sleep(0.15)

    def wake_sequence(self):
        """Wakes like opening eyes & blinking."""
        for duty in np.linspace(0, 1, 20):
            self.tcs_led.value = duty
            time.sleep(0.04)
        for _ in range(3):
            self.tcs_led.value = 0
            time.sleep(0.15)
            self.tcs_led.value = 1
            time.sleep(0.15)
        time.sleep(1)

# ----------------------------------------------------------------------------- 
# ROBOT
# ----------------------------------------------------------------------------- 
class Robot:
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(Config.C_PIN, GPIO.OUT)
        GPIO.setup(Config.NC_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.output(Config.C_PIN, GPIO.HIGH)

        self.motor      = MotorController(Config.IN1, Config.IN2, Config.ENA, Config.PWM_MOTOR_FREQ)
        self.servo      = ServoController(Config.SERVO_PIN, Config.PWM_SERVO_FREQ,
                                          min_deg=Config.STEER_MIN_DEG,
                                          max_deg=Config.STEER_MAX_DEG)

        # gpiozero LEDs + Button
        self.button_led = PWMLED(Config.BUTTON_LED_PIN, frequency=100)
        self.tcs_led    = PWMLED(Config.TCS_LED_PIN, frequency=100)
        self.button     = Button(Config.BUTTON_PIN)

        i2c = busio.I2C(board.SCL, board.SDA)
        self.mux = TCA9548A(i2c)

        # Sensors
        self.sensors = SensorManager(self.mux)
        self.sensors.start()

        # TCS color sensor
        self.mux.select_channel(Config.SENSOR_CHANNELS['TCS'])
        self.color_sensor = adafruit_tcs34725.TCS34725(i2c)
        self._color_filter = RollingFilter(3)

        # Robot "state"
        self.orientation = None
        self.lap_count = 0
        self.color_seen = 0

        # Robot "personality" LED animation
        self.led_anim = LEDAnimation(self.button_led, self.tcs_led)
        self.led_anim.start()

    def wait_for_button(self):
        logger.info("Waiting for button press...")
        self.button.wait_for_press()
        logger.info("Button pressed! Running wake animation...")
        self.led_anim.wake_sequence()
        time.sleep(2)

    def detect_orientation(self):
        """Decide CW/CCW by dominant color (personality moment)."""
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
                 (self.orientation == 'CW' and r_avg > g_avg and r_avg > b_avg))
        self.color_seen = self.color_seen + 1 if match else 0
        if self.color_seen >= Config.COLOR_COUNT_FOR_LAP:
            self.lap_count += 1
            self.color_seen = 0
            logger.info("🏁 Lap %d completed!", self.lap_count)

    def main_loop(self):
        try:
            self.wait_for_button()
            while self.orientation is None:
                self.detect_orientation()

            while self.lap_count < Config.TOTAL_LAPS:
                self.check_lap()

                front = self.sensors.range_cm('F')
                fl = self.sensors.range_cm('FL')
                fr = self.sensors.range_cm('FR')
                rl = self.sensors.range_cm('RL')
                rr = self.sensors.range_cm('RR')

                # Steering logic (optimized but same behavior)
                front_diff = fr - fl
                rear_diff = rr - rl if self.orientation == 'CCW' else rl - rr

                side_correction = 0
                if fl < Config.SIDE_THRESHOLD_CM or fr < Config.SIDE_THRESHOLD_CM:
                    side_correction = (fr - fl) * 0.5
                if rl < Config.SIDE_THRESHOLD_CM or rr < Config.SIDE_THRESHOLD_CM:
                    side_correction += ((rr - rl) if self.orientation == 'CCW' else (rl - rr)) * 0.3

                correction = (front_diff * 0.7 + rear_diff * 0.3) if front < Config.FRONT_THRESHOLD_CM \
                             else (front_diff * 0.4 + rear_diff * 0.6)
                correction += side_correction

                steer = np.clip(90 + correction, Config.STEER_MIN_DEG, Config.STEER_MAX_DEG)
                self.servo.set_angle(steer)

                # Motor control
                if front < 20:
                    self.motor.stop()
                elif front < Config.FRONT_THRESHOLD_CM:
                    speed = max(10, (front / Config.FRONT_THRESHOLD_CM) * 35)
                    self.motor.forward(speed)
                else:
                    self.motor.forward(35)

                time.sleep(0.05)

        except Exception:
            logger.exception("Unhandled exception in main_loop")
        finally:
            self.shutdown()

    def shutdown(self):
        logger.info("Shutdown initiated.")
        self.motor.stop()
        self.led_anim.stop()
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
