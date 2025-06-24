import time
import board
import busio
import adafruit_vl53l0x
import RPi.GPIO as GPIO
from gpiozero import DistanceSensor
from pixy2 import Pixy2

# === GPIO pin definitions ===
# VL53L0X XSHUT pins
XSHUT_RF = 4
XSHUT_LF = 5
XSHUT_F = 6

# Ultrasonic sensors (back) labeled as requested
sensor_lb = DistanceSensor(echo=20, trigger=21)
sensor_rb = DistanceSensor(echo=26, trigger=16)

# Motor and steering pins
IN1 = 17
IN2 = 27
ENA = 18
SERVO_PIN = 12

# Pixy2 color signatures (must match PixyMon config)
RED_SIG = 1
GREEN_SIG = 2

# === Global variables ===
laps_completed = 0

# Initialize GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Setup XSHUT pins as outputs, keep sensors off initially
for pin in [XSHUT_RF, XSHUT_LF, XSHUT_F]:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)
time.sleep(0.1)

# Setup motor pins
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Initialize PWM for motor and servo
pwm_motor = GPIO.PWM(ENA, 100)     # 100 Hz
pwm_servo = GPIO.PWM(SERVO_PIN, 50) # 50 Hz servo

pwm_motor.start(0)
pwm_servo.start(7.5)  # Neutral position (~90 deg)

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Power on and assign unique addresses to VL53L0X sensors
vl53_addresses = [0x30, 0x31, 0x32]
vl53_sensors = []

for i, pin in enumerate([XSHUT_RF, XSHUT_LF, XSHUT_F]):
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(0.1)
    sensor = adafruit_vl53l0x.VL53L0X(i2c)
    sensor.set_address(vl53_addresses[i])
    vl53_sensors.append(sensor)

sensor_rf = vl53_sensors[0]
sensor_lf = vl53_sensors[1]
sensor_f = vl53_sensors[2]

# Initialize Pixy2
pixy = Pixy2()

# --- Motor control functions ---
def set_servo_angle(angle):
    # Angle 0-180 degrees for rear steering servo
    if not 0 <= angle <= 180:
        print("Servo angle out of range")
        return
    duty = 2 + (angle / 18)  # Convert angle to duty cycle approx.
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
    pwm_motor.ChangeDutyCycle(0)

# --- Logic functions ---
def detect_block():
    blocks = pixy.get_blocks()
    if blocks:
        # Pick biggest block (by width)
        block = max(blocks, key=lambda b: b.width)
        return block
    return None

def decide_turn_side(block):
    center_x = 158  # Pixy2 camera center x
    if block.x < center_x:
        return "right"  # Block left, pass on right
    else:
        return "left"   # Block right, pass on left

def obstacle_in_front():
    # Use VL53L0X sensors to check for obstacles closer than 25 cm
    dist_f = sensor_f.range / 10
    dist_lf = sensor_lf.range / 10
    dist_rf = sensor_rf.range / 10
    if dist_f < 25 or dist_lf < 20 or dist_rf < 20:
        return True
    return False

def obstacle_at_back_side():
    # Check ultrasonic sensors at back, less than 15cm means obstacle
    dist_lb = sensor_lb.distance * 100
    dist_rb = sensor_rb.distance * 100
    if dist_lb < 15:
        return "left"
    elif dist_rb < 15:
        return "right"
    else:
        return None

def go_around_block(side):
    # Steering angle for rear steering
    angle_left = 45
    angle_right = 135
    angle_straight = 90

    if side == "left":
        set_servo_angle(angle_left)
    else:
        set_servo_angle(angle_right)

    forward(30)
    # Move forward while block visible
    timeout = time.time() + 5  # max 5 seconds to avoid infinite loop
    while time.time() < timeout:
        block = detect_block()
        if block is None:
            break  # Block passed
        # Check walls while going around
        if obstacle_in_front():
            stop()
            reverse(30)
            time.sleep(0.5)
            stop()
            break
        # Check back obstacles and steer away
        back_obs = obstacle_at_back_side()
        if back_obs == "left":
            set_servo_angle(angle_right)
        elif back_obs == "right":
            set_servo_angle(angle_left)

        time.sleep(0.1)
    # Straighten steering after passing block
    set_servo_angle(angle_straight)

def main_loop():
    global laps_completed
    try:
        while laps_completed < 3:
            block = detect_block()
            if block:
                print(f"Block detected: sig {block.signature}, pos {block.x}, size {block.width}")
                side = decide_turn_side(block)
                print(f"Going around block on the {side} side")
                go_around_block(side)
            else:
                # No block, move forward and avoid obstacles
                if obstacle_in_front():
                    print("Obstacle ahead! Stopping and reversing.")
                    stop()
                    reverse(30)
                    time.sleep(0.5)
                    stop()
                else:
                    set_servo_angle(90)
                    forward(40)

                # Check back obstacles and steer slightly if needed
                back_obs = obstacle_at_back_side()
                if back_obs == "left":
                    print("Back obstacle left side, steering right.")
                    set_servo_angle(135)
                elif back_obs == "right":
                    print("Back obstacle right side, steering left.")
                    set_servo_angle(45)
                else:
                    set_servo_angle(90)

            # Example lap counting logic placeholder
            # Increment laps_completed using your actual sensor / logic
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping robot")

    finally:
        stop()
        pwm_servo.stop()
        pwm_motor.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main_loop()
