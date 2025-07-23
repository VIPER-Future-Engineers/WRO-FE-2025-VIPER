import time
import board
import busio
import adafruit_vl53l0x
import RPi.GPIO as GPIO

# === GPIO Assignments ===
XSHUT_LF = 24
XSHUT_RF = 23
SERVO_PIN = 19

TRIG_UL_1 = 5
ECHO_UL_1 = 6
TRIG_UL_2 = 17
ECHO_UL_2 = 27

# === GPIO Setup ===
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup([XSHUT_LF, XSHUT_RF, TRIG_UL_1, TRIG_UL_2], GPIO.OUT)
GPIO.setup([ECHO_UL_1, ECHO_UL_2], GPIO.IN)
GPIO.output([TRIG_UL_1, TRIG_UL_2, XSHUT_LF, XSHUT_RF], GPIO.LOW)

GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm_servo = GPIO.PWM(SERVO_PIN, 50)
pwm_servo.start(7.5)

time.sleep(0.2)

# === I2C Setup ===
i2c = busio.I2C(board.SCL, board.SDA)

# === TOF Sensor Initialization ===
def init_tof_sensors():
    try:
        GPIO.output(XSHUT_LF, GPIO.HIGH)
        time.sleep(1.0)
        sensor_lf = adafruit_vl53l0x.VL53L0X(i2c)
        sensor_lf.set_address(0x30)

        GPIO.output(XSHUT_RF, GPIO.HIGH)
        time.sleep(1.0)
        sensor_rf = adafruit_vl53l0x.VL53L0X(i2c)
        sensor_rf.set_address(0x31)

        sensor_lf.start_continuous()
        sensor_rf.start_continuous()
        sensor_lf.measurement_timing_budget = 50000
        sensor_rf.measurement_timing_budget = 50000

        return sensor_lf, sensor_rf

    except Exception as e:
        print(f"TOF sensor init failed: {e}")
        GPIO.cleanup()
        exit(1)

sensor_lf, sensor_rf = init_tof_sensors()

# === Helper Functions ===
def set_servo_angle(angle):
    duty = 2.5 + (angle / 18)
    pwm_servo.ChangeDutyCycle(duty)

def safe_read(sensor, samples=3):
    readings = []
    for _ in range(samples):
        try:
            readings.append(sensor.range)
        except:
            readings.append(0)
        time.sleep(0.05)
    return sum(readings) // len(readings)

def read_ultrasonic(trig, echo):
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    timeout = time.time() + 0.05
    while GPIO.input(echo) == 0:
        if time.time() > timeout:
            return 0
    start = time.time()

    timeout = time.time() + 0.05
    while GPIO.input(echo) == 1:
        if time.time() > timeout:
            return 0
    end = time.time()

    distance = ((end - start) * 34300) / 2
    return round(distance, 1)

# === Main Loop ===
def main_loop():
    try:
        while True:
            dist_lf = safe_read(sensor_lf) / 10
            dist_rf = safe_read(sensor_rf) / 10
            dist_ul_1 = read_ultrasonic(TRIG_UL_1, ECHO_UL_1)
            dist_ul_2 = read_ultrasonic(TRIG_UL_2, ECHO_UL_2)

            print(f"TOF LF: {dist_lf:.1f} cm | TOF RF: {dist_rf:.1f} cm")
            print(f"Ultrasonic 1: {dist_ul_1:.1f} cm | Ultrasonic 2: {dist_ul_2:.1f} cm")

            if dist_ul_1 < 20 or dist_ul_2 < 20:
                print("🧱 Obstacle detected — steering CENTER")
                set_servo_angle(90)
            elif abs(dist_lf - dist_rf) > 5:
                if dist_lf > dist_rf:
                    print("↪ Steering LEFT")
                    set_servo_angle(45)
                else:
                    print("↩ Steering RIGHT")
                    set_servo_angle(135)
            else:
                print("⬆ Balanced TOF — steering CENTER")
                set_servo_angle(90)

            time.sleep(0.3)

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        pwm_servo.stop()
        GPIO.cleanup()
        

if __name__ == "__main__":
    main_loop()


