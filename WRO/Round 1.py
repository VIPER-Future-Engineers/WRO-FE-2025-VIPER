import time
import board
import busio
import threading
import collections
import logging

import adafruit_vl53l0x
import adafruit_tcs34725
import RFRobot_I2C_Multiplexer as rf_mux  # <-- RF Robot multiplexer lib
import RPi.GPIO as GPIO

# -----------------------------------------------------------------------------
# CONFIGURATION (Easy‐change section)
# -----------------------------------------------------------------------------
class Config:
    # GPIO pins
    IN1            = 9
    IN2            = 11
    ENA            = 10
    SERVO_PIN      = 25
    BUTTON_PIN     = 23    # AL2 Halo Switch NO
    BUTTON_LED_PIN = 27     # AL2 Halo Switch LED
    BUTTON_C_PIN = 22
    BUTTON_NC_PIN = 24
    TCS_LED_PIN    = 17    # TCS34725 onboard LED

    # PWM frequencies (Hz)
    PWM_MOTOR_FREQ      = 1000
    PWM_SERVO_FREQ      = 50
    PWM_BUTTON_LED_FREQ = 500
    PWM_TCS_LED_FREQ    = 500

    # VL53L0X multiplexer channels
    SENSOR_CHANNELS = {
        'FR': 1,   # front‐right
        'FL': 2,   # front‐left
        'F':  3,   # front-center
        'RL': 4,   # rear-left
        'RR': 5,   # rear-right
        'TCS': 0,  # color sensor
    }

    # ToF timing budgets (µs)
    BUDGET_FRONT_US = 20000
    BUDGET_REAR_US  = 33000

    # Driving thresholds
    FRONT_THRESHOLD_CM = 40.0
    STEER_LEFT_DEG     = 45
    STEER_RIGHT_DEG    = 135
    STEER_STRAIGHT_DEG = 90

    # Lap logic
    COLOR_COUNT_FOR_LAP = 4
    TOTAL_LAPS          = 3

# -----------------------------------------------------------------------------
# LOGGING SETUP
# -----------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
logger = logging.getLogger(__name__)

# -----------------------------------------------------------------------------
# HELPER CLASSES
# -----------------------------------------------------------------------------
class RollingFilter:
    """Maintain a rolling average over the last N samples (in mm)."""
    def __init__(self, size: int = 3):
        self._buf = collections.deque(maxlen=size)

    def add(self, sample: float) -> float:
        self._buf.append(sample)
        return sum(self._buf) / len(self._buf)


class MotorController:
    """DC motor via H-bridge PWM."""
    def __init__(self, in1: int, in2: int, pwm_pin: int, freq: int):
        GPIO.setup(in1, GPIO.OUT)
        GPIO.setup(in2, GPIO.OUT)
        self._in1 = in1
        self._in2 = in2
        self._pwm = GPIO.PWM(pwm_pin, freq)
        self._pwm.start(0)

    def forward(self, speed: float) -> None:
        GPIO.output(self._in1, GPIO.LOW)
        GPIO.output(self._in2, GPIO.HIGH)
        self._pwm.ChangeDutyCycle(speed)

    def stop(self) -> None:
        GPIO.output(self._in1, GPIO.HIGH)
        GPIO.output(self._in2, GPIO.HIGH)
        self._pwm.ChangeDutyCycle(0)


class ServoController:
    """Standard hobby servo control."""
    def __init__(self, pin: int, freq: int):
        GPIO.setup(pin, GPIO.OUT)
        self._pwm = GPIO.PWM(pin, freq)
        self._pwm.start(7.5)  # center

    def set_angle(self, angle: float) -> None:
        if 0 <= angle <= 180:
            duty = 2 + (angle / 18)
            self._pwm.ChangeDutyCycle(duty)
        else:
            logger.warning("Servo angle out of range: %s", angle)


class LEDController:
    """PWM brightness for an LED."""
    def __init__(self, pin: int, freq: int):
        GPIO.setup(pin, GPIO.OUT)
        self._pwm = GPIO.PWM(pin, freq)
        self._pwm.start(0)

    def set_duty(self, duty: float) -> None:
        duty = max(0, min(100, duty))
        self._pwm.ChangeDutyCycle(duty)

    def stop(self) -> None:
        self._pwm.ChangeDutyCycle(0)


class SensorManager:
    """VL53L0X array behind an RF Robot I2C multiplexer."""
    def __init__(self, mux: rf_mux.I2C_Multiplexer):
        self._sensors: dict[str, adafruit_vl53l0x.VL53L0X] = {}
        self._filters: dict[str, RollingFilter] = {}
        for name, ch in Config.SENSOR_CHANNELS.items():
            if name == 'TCS':
                continue
            mux.channel_enable(ch)
            sens = adafruit_vl53l0x.VL53L0X(mux.i2c)
            budget = (
                Config.BUDGET_FRONT_US
                if name.startswith('F')
                else Config.BUDGET_REAR_US
            )
            sens.measurement_timing_budget = budget
            sens.start_continuous()
            self._sensors[name] = sens
            self._filters[name] = RollingFilter(3)
            mux.channel_disable(ch)

    def range_cm(self, name: str) -> float:
        """Return filtered distance in centimeters."""
        ch = Config.SENSOR_CHANNELS[name]
        mux.channel_enable(ch)
        mm = self._sensors[name].range
        mux.channel_disable(ch)
        avg_mm = self._filters[name].add(mm)
        return avg_mm / 10.0


class HeartbeatThread:
    """
    Background heartbeat animation.
    Stops gracefully on run_event.clear() or interrupt_event.set().
    """
    def __init__(
        self,
        led: LEDController,
        tcs_led: LEDController,
        run_event: threading.Event,
        interrupt_event: threading.Event,
        mode_event: threading.Event
    ):
        self.led = led
        self.tcs_led = tcs_led
        self.run_evt = run_event
        self.int_evt = interrupt_event
        self.mode_evt = mode_event
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def _beat_once(self, strength: float, speed: float) -> None:
        pattern = [(100,0.1),(30,0.1),(80,0.08),(20,0.1)]
        for duty, delay in pattern:
            self.led.set_duty(duty * strength)
            time.sleep(delay * speed)
        self.led.set_duty(0)
        time.sleep(0.6 * speed)

    def _fade_out(self, start: int, strength: float, speed: float) -> None:
        logger.info("Fading heartbeat…")
        for i in range(start, 10):
            decay = 1.0 - (i * 0.1)
            tempo = speed * (1.0 + i * 0.1)
            self._beat_once(strength * decay, tempo)
        # TCS LED flicker then off
        for duty in [40,100,30,90]:
            self.tcs_led.set_duty(duty)
            time.sleep(0.12)
        for duty in range(70, -1, -10):
            self.tcs_led.set_duty(duty)
            time.sleep(0.15)
        self.tcs_led.set_duty(0)
        logger.info("Heartbeat and TCS LED off.")

    def _run(self) -> None:
        logger.info("Heartbeat thread started (idle).")
        while self.run_evt.is_set() and not self.int_evt.is_set():
            active = self.mode_evt.is_set()
            strength, speed, cycles = (
                (1.0, 0.7, 12) if active else (0.6, 1.2, 8)
            )
            for i in range(cycles):
                if not self.run_evt.is_set() or self.int_evt.is_set():
                    self._fade_out(i, strength, speed)
                    return
                self._beat_once(strength, speed)
        # normal exit fade
        strength, speed = (
            (1.0, 0.7) if self.mode_evt.is_set() else (0.6, 1.2)
        )
        self._fade_out(0, strength, speed)
        logger.info("Heartbeat thread exiting.")


# -----------------------------------------------------------------------------
# ROBOT CLASS
# -----------------------------------------------------------------------------
class Robot:
    """Runs drive, sensors, LEDs, and lap logic end-to-end."""
    def __init__(self) -> None:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        # Motor, servo, LEDs
        self.motor      = MotorController(
            Config.IN1, Config.IN2, Config.ENA, Config.PWM_MOTOR_FREQ
        )
        self.servo      = ServoController(
            Config.SERVO_PIN, Config.PWM_SERVO_FREQ
        )
        self.button_led = LEDController(
            Config.BUTTON_LED_PIN, Config.PWM_BUTTON_LED_FREQ
        )
        self.tcs_led    = LEDController(
            Config.TCS_LED_PIN, Config.PWM_TCS_LED_FREQ
        )

        # I2C + RF Robot multiplexer
        i2c = busio.I2C(board.SCL, board.SDA)
        global mux
        mux = rf_mux.I2C_Multiplexer(i2c, address=0x70)

        # Sensors
        self.sensors      = SensorManager(mux)
        self.color_sensor = adafruit_tcs34725.TCS34725(
            mux.i2c
        )

        # State & threading events
        self.orientation     = None  # 'CW' or 'CCW'
        self.lap_count       = 0
        self.color_seen      = 0

        self.run_event       = threading.Event()
        self.interrupt_event = threading.Event()
        self.mode_event      = threading.Event()

        self.heartbeat = HeartbeatThread(
            self.button_led,
            self.tcs_led,
            self.run_event,
            self.interrupt_event,
            self.mode_event
        )

        self._setup_button()

    def _setup_button(self) -> None:
        GPIO.setup(Config.BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(
            Config.BUTTON_PIN, GPIO.FALLING,
            callback=self._button_press, bouncetime=50
        )
        GPIO.add_event_detect(
            Config.BUTTON_PIN, GPIO.RISING,
            callback=self._button_release, bouncetime=50
        )
        self._press_time = None

    def _button_press(self, channel: int) -> None:
        self._press_time = time.time()

    def _button_release(self, channel: int) -> None:
        duration = time.time() - (self._press_time or 0)
        if duration >= 3.0:
            logger.info("Soft reset triggered.")
            self.soft_reset()
        else:
            if not self.run_event.is_set():
                self.start_run()
            else:
                logger.info("Interrupting run.")
                self.interrupt_event.set()
                self.run_event.clear()
                self.motor.stop()

    def wake_sequence(self) -> None:
        """Fade in halo + TCS LED and blink like ‘eyes opening.’"""
        logger.info("Wake: opening eyes…")
        self.button_led.set_duty(100)
        for d in range(0, 101, 5):
            self.tcs_led.set_duty(d)
            time.sleep(0.04)
        for _ in range(3):
            self.tcs_led.set_duty(0); time.sleep(0.15)
            self.tcs_led.set_duty(100); time.sleep(0.15)
        self.tcs_led.set_duty(0)
        time.sleep(1)

    def detect_orientation(self) -> None:
        """Once-only: blue→CCW, orange→CW."""
        logger.info("Detecting orientation (blue=CCW, orange=CW)…")
        while self.orientation is None:
            r, g, b = self.color_sensor.color_rgb_bytes
            if b > r and b > g and b > 50:
                self.orientation = 'CCW'
            elif r > g and r > b and r > 50:
                self.orientation = 'CW'
            else:
                time.sleep(0.1)
        logger.info("Orientation set to %s", self.orientation)

    def check_lap(self) -> None:
        """Count 4× consecutive orientation-color readings → lap++."""
        r, g, b = self.color_sensor.color_rgb_bytes
        match = (
            (self.orientation == 'CCW' and b > r and b > g) or
            (self.orientation == 'CW'  and r > g and r > b)
        )
        self.color_seen = self.color_seen + 1 if match else 0
        if self.color_seen >= Config.COLOR_COUNT_FOR_LAP:
            self.lap_count += 1
            self.color_seen = 0
            logger.info("🏁 Lap %d completed!", self.lap_count)

    def start_run(self) -> None:
        """Begin wake + idle heartbeat + active run."""
        self.run_event.set()
        self.interrupt_event.clear()
        self.mode_event.clear()     # idle
        self.wake_sequence()
        self.heartbeat.start()
        time.sleep(5)               # startup delay
        self.mode_event.set()       # active
        logger.info("Autonomous run started")

    def soft_reset(self) -> None:
        """Clear state, LEDs, and lap counters."""
        self.run_event.clear()
        self.interrupt_event.clear()
        self.mode_event.clear()
        self.button_led.set_duty(0)
        self.tcs_led.set_duty(0)
        self.lap_count   = 0
        self.orientation = None
        self.color_seen  = 0

    def shutdown(self) -> None:
        """Stop motors/LEDs and cleanup GPIO."""
        logger.info("Shutdown initiated.")
        self.run_event.clear()
        self.interrupt_event.set()
        self.motor.stop()
        self.button_led.stop()
        self.tcs_led.stop()
        GPIO.cleanup()

    def main_loop(self) -> None:
        """Drive, avoid walls, and count laps until complete."""
        try:
            self.detect_orientation()
            while self.lap_count < Config.TOTAL_LAPS:
                if not self.run_event.is_set():
                    time.sleep(0.1)
                    continue

                self.check_lap()

                front = self.sensors.range_cm('F')
                if front < Config.FRONT_THRESHOLD_CM:
                    fl = self.sensors.range_cm('FL')
                    fr = self.sensors.range_cm('FR')
                    steer = (
                        Config.STEER_LEFT_DEG
                        if (fl > fr) == (self.orientation == 'CCW')
                        else Config.STEER_RIGHT_DEG
                    )
                else:
                    rl = self.sensors.range_cm('RL')
                    rr = self.sensors.range_cm('RR')
                    if self.orientation == 'CCW':
                        steer = (
                            Config.STEER_LEFT_DEG if rl > rr
                            else Config.STEER_RIGHT_DEG
                        )
                    else:
                        steer = (
                            Config.STEER_RIGHT_DEG if rr > rl
                            else Config.STEER_LEFT_DEG
                        )

                self.servo.set_angle(steer)
                self.motor.forward(50)
                time.sleep(0.05)

        except Exception:
            logger.exception("Unhandled exception in main_loop")
        finally:
            self.shutdown()


def main() -> None:
    robot = Robot()
    robot.main_loop()


if __name__ == "__main__":
    main()
