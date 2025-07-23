import time
import board
import busio
import RPi.GPIO as GPIO
import adafruit_vl53l0x
import I2C

# Copyright (c) 2025 Jadon
# Unauthorized use is prohibited.

# Setup I2C and VL53L0X sensor
i2c = busio.I2C(board.SCL, board.SDA)
tof = adafruit_vl53l0x.VL53L0X(i2c)

# Motor and steering control pins
IN1 = 17
IN2 = 27
ENA = 18
SERVO_PIN = 12

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Initialize PWM
pwm_servo = GPIO.PWM(SERVO_PIN, 50)  # Servo: 50 Hz
pwm_motor = GPIO.PWM(ENA, 100)       # Motor: 100 Hz

# Start PWM outputs
pwm_servo.start(7.5)  # Neutral position (90 deg)
pwm_motor.start(0)

# Global state
colour_count = 0
lap = 0
orientation = "None"

def set_angle(angle):
    """Rotate servo to specified angle (0-180)."""
    if not 0 <= angle <= 180:
        print("Angle out of range.")
        return
    duty = 2 + (angle / 16)
    pwm_servo.ChangeDutyCycle(duty)


def reverse(speed):
    if not 0 <= speed <= 100:
        print("Speed out of range.")
        return
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm_motor.ChangeDutyCycle(speed)


def forward(speed):
    if not 0 <= speed <= 100:
        print("Speed out of range.")
        return
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm_motor.ChangeDutyCycle(speed)


def stop():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.HIGH)


def coast():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

# Copyright (c) 2025 Jadon
# Unauthorized use is prohibited.

def direction():
    global colour_count, orientation
    distance = tof.range  # Read in mm

    if distance > 200:  # 20 cm
        while tof.range > 200:
            forward(15)
    else:
        stop()
        # Simulated color sensor condition
        if '''colour sensor''' == '''blue colour value''':
            orientation = 'counterclockwise'
            colour_count += 1
            print("counterclockwise")
        elif '''colour sensor''' == '''orange colour value''':
            orientation = 'clockwise'
            colour_count += 1
            print("clockwise")
        else:
            print("Reading Error")


def setup():
    global colour_count, lap
    colour_count = 0
    lap = 0
    set_angle(90)
    direction()


def perform_sequence_clockwise():
    distance3 = tof.range / 10  # convert mm to cm
    print(f"Front Distance: {distance3:.1f} cm")

    max_range = 100
    min_range = 40

    if min_range <= distance3 <= max_range:
        angle = 90 + ((max_range + distance3) / (max_range + min_range) * 25)
        set_angle(max(angle, 107.5))
        forward(20)
    elif distance3 < min_range:
        set_angle(107.5)
        forward(20)
    else:
        set_angle(90)
        forward(15)

    time.sleep(0.0001)


def perform_sequence_counterclockwise():
    distance3 = tof.range / 10  # convert mm to cm
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


try:
    setup()
    if orientation == 'clockwise':
        while True:
            perform_sequence_clockwise()
    elif orientation == 'counterclockwise':
        while True:
            perform_sequence_counterclockwise()

except KeyboardInterrupt:
    print("Interrupted by user.") 

finally:
    pwm_motor.stop() 
    pwm_servo.stop() 
    GPIO.cleanup() 
