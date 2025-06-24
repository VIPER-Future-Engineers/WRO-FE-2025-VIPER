import time
import board
import busio
import adafruit_vl53l0x
import RPi.GPIO as GPIO
from gpiozero import DistanceSensor

# === Global Counters ===
colour_count = 0
lap = 0
orientation = ""

# === GPIO Pin Setup ===
XSHUT_RF = 4   # Right Front
XSHUT_LF = 5   # Left Front
XSHUT_F  = 6   # Front

XSHUT_PINS = [XSHUT_RF, XSHUT_LF, XSHUT_F]
I2C_ADDRESSES = [0x30, 0x31, 0x32]

# === Initialize GPIO for XSHUT ===
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
for pin in XSHUT_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)  # Shutdown all VL53L0X initially
time.sleep(0.1)

# === Initialize I2C and Sensors ===
i2c = busio.I2C(board.SCL, board.SDA)
sensors = []

for i, pin in enumerate(XSHUT_PINS):
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(0.1)
    sensor = adafruit_vl53l0x.VL53L0X(i2c)
    sensor.set_address(I2C_ADDRESSES[i])
    sensors.append(sensor)

# Assign meaningful names
sensor_rf = sensors[0]  # Right Front VL53L0X
sensor_lf = sensors[1]  # Left Front VL53L0X
sensor_f  = sensors[2]  # Front VL53L0X

# === Rear Ultrasonic Sensors ===
sensor_rb = DistanceSensor(echo=13, trigger=19)  # Right Back
sensor_lb = DistanceSensor(echo=20, trigger=21)  # Left Back

# === Motor and Steering Control ===
IN1 = 17
IN2 = 27
ENA = 18
SERVO_PIN = 12

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm_servo = GPIO.PWM(SERVO_PIN, 50)
pwm_motor = GPIO.PWM(ENA, 100)
pwm_servo.start(90)
pwm_motor.start(0)

# === Helper Functions ===
def set_angle(angle):
    if not 0 <= angle <= 180:
        print("Angle out of range. Must be 0–180°.")
        return
    duty = 2 + (angle / 16)
    pwm_servo.ChangeDutyCycle(duty)

def forward(speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm_motor.ChangeDutyCycle(speed)

def reverse(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm_motor.ChangeDutyCycle(speed)

def stop():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.HIGH)

def coast():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

def direction():
    global orientation, colour_count, lap
    distance_front = sensor_f.range / 10  # in cm

    if distance_front > 20:
        while distance_front > 20:
            forward(15)
            distance_front = sensor_f.range / 10
    else:
        stop()
        # Simulated color sensor check
        detected_colour = "blue"  # Replace with real sensor
        if detected_colour == "blue":
            orientation = 'counterclockwise'
            colour_count += 1
            print('counterclockwise')
        elif detected_colour == "orange":
            orientation = 'clockwise'
            colour_count += 1
            print('clockwise')
        else:
            print("Reading Error")

    if colour_count == 4:
        lap += 1
        colour_count = 0
        time.sleep(0.1)

def setup():
    global colour_count, lap
    colour_count = 0
    lap = 0
    set_angle(90)
    direction()

def perform_sequence_clockwise():
    distance1 = sensor_rf.range / 10
    distance2 = sensor_rb.distance * 100
    distance3 = sensor_f.range / 10
    distance4 = sensor_lf.range / 10
    distance5 = sensor_lb.distance * 100

    print(f"Front Distance: {distance3:.1f} cm")

    max_range = 100
    min_range = 40

    if min_range <= distance3 <= max_range:
        angle = 90 + ((max_range + distance3) / (max_range + min_range) * 25)
        set_angle(min(angle, 107.5))
        forward(20)
    elif distance3 < min_range:
        set_angle(107.5)
        forward(20)
    else:
        set_angle(90)
        forward(15)

    time.sleep(0.0001)

def perform_sequence_counterclockwise():
    distance1 = sensor_rf.range / 10
    distance2 = sensor_rb.distance * 100
    distance3 = sensor_f.range / 10
    distance4 = sensor_lf.range / 10
    distance5 = sensor_lb.distance * 100

    print(f"Front Distance: {distance3:.1f} cm")

    max_range = 100
    min_range = 40

    if min_range <= distance3 <= max_range:
        angle = 90 - ((max_range - distance3) / (max_range - min_range) * 25)
        set_angle(max(angle, 65))
        forward(20)
    elif distance3 < min_range:
        set_angle(64.5)
        forward(20)
    else:
        set_angle(90)
        forward(20)

    time.sleep(0.0001)

# === Main Program ===
try:
    setup()
    while lap < 3:
        if orientation == 'clockwise':
            perform_sequence_clockwise()
        elif orientation == 'counterclockwise':
            perform_sequence_counterclockwise()

    print("Laps complete")

finally:
    pwm_servo.stop()
    pwm_motor.stop()
    GPIO.cleanup()
