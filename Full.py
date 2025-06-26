import time                                                             # imports the library for time
import board                                                            # imports the library for I2C (1)
import busio                                                            # imports the library for I2C (2)
import adafruit_vl53l0x                                                 # imports the library for TOF Sensors (Time of flight)
import RPi.GPIO as GPIO                                                 # imports the library for Motors
from gpiozero import DistanceSensor                                     # imports the library for Ultrasonic sensors
from pixy2 import Pixy2                                                 # imports the library for Pixy camera

# GPIO pin definitions 
# VL53L0X XSHUT pins - used to power on/off sensors individually
XSHUT_RF = 4
XSHUT_LF = 5
XSHUT_F = 6

# Ultrasonic sensors GPIO pins for back left and back right
sensor_lb = DistanceSensor(echo=20, trigger=21)
sensor_rb = DistanceSensor(echo=26, trigger=16)

# Motor and steering pins
IN1 = 17              # Motor input 1
IN2 = 27              # Motor input 2
ENA = 18              # Motor enable pin (PWM)
SERVO_PIN = 12        # Rear steering servo PWM pin

# Pixy2 color signatures (must match PixyMon config)
GREEN_SIG = 1         # Signature 1 is green
RED_SIG = 2           # Signature 2 is red

# Global Variables
laps_completed = 0    # Counter for laps completed (placeholder)

# Initialize GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Setup XSHUT pins as outputs, keep sensors off initially
for pin in [XSHUT_RF, XSHUT_LF, XSHUT_F]:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)
time.sleep(0.1)  # Short delay to ensure sensors are powered down

# Setup motor pins as outputs
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Initialize PWM for motor and servo
pwm_motor = GPIO.PWM(ENA, 100)        # Motor PWM at 100 Hz frequency
pwm_servo = GPIO.PWM(SERVO_PIN, 50)   # Servo PWM at 50 Hz frequency

pwm_motor.start(0)                     # Start motor PWM with 0% duty cycle (stopped)
pwm_servo.start(7.5)                   # Start servo PWM at neutral position (~90 degrees)

# Initialize I2C bus for sensors
i2c = busio.I2C(board.SCL, board.SDA)

# Power on and assign unique I2C addresses to VL53L0X sensors
vl53_addresses = [0x30, 0x31, 0x32]   # Unique I2C addresses for each sensor
vl53_sensors = []

for i, pin in enumerate([XSHUT_RF, XSHUT_LF, XSHUT_F]):
    GPIO.output(pin, GPIO.HIGH)        # Power on each VL53L0X sensor one by one
    time.sleep(0.1)
    sensor = adafruit_vl53l0x.VL53L0X(i2c)
    sensor.set_address(vl53_addresses[i])  # Assign new I2C address to avoid conflicts
    vl53_sensors.append(sensor)

# Assign sensors for easier reference
sensor_rf = vl53_sensors[0]  # Right front sensor
sensor_lf = vl53_sensors[1]  # Left front sensor
sensor_f = vl53_sensors[2]   # Front center sensor

# Initialize Pixy2 camera
pixy = Pixy2()

# Motor control functions
def set_servo_angle(angle):
    """
    Set rear steering servo angle (0-180 degrees).
    Converts angle to PWM duty cycle and updates servo.
    """
    if not 0 <= angle <= 180:
        print("Servo angle out of range")
        return
    duty = 2 + (angle / 18)  # Approximate conversion from angle to duty cycle
    pwm_servo.ChangeDutyCycle(duty)

def forward(speed):
    """
    Move robot forward at specified speed (PWM duty cycle).
    IN1 LOW, IN2 HIGH for forward rotation.
    """
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm_motor.ChangeDutyCycle(speed)

def reverse(speed):
    """
    Move robot backward at specified speed (PWM duty cycle).
    IN1 HIGH, IN2 LOW for reverse rotation.
    """
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm_motor.ChangeDutyCycle(speed)

def stop():
    """
    Stop the motor by setting both IN1 and IN2 HIGH and PWM to 0.
    """
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.HIGH)
    pwm_motor.ChangeDutyCycle(0)

# Detection and decision functions
def detect_closest_color_block():
    """
    Detect blocks from Pixy2 and return the closest block with signature green or red.
    Closest block is assumed to have the largest width.
    Returns None if no relevant block detected.
    """
    blocks = pixy.get_blocks()
    if not blocks:
        return None
    # Filter blocks for green or red signatures only
    color_blocks = [b for b in blocks if b.signature in (GREEN_SIG, RED_SIG)]
    if not color_blocks:
        return None
    # Return block with maximum width (closest)
    closest_block = max(color_blocks, key=lambda b: b.width)
    return closest_block

def decide_turn_side_by_signature(block):
    """
    Decide which side to go around the block based on its signature.
    Green (signature 1) -> left side
    Red (signature 2) -> right side
    """
    if block.signature == RED_SIG:
        return "right"
    elif block.signature == GREEN_SIG:
        return "left"
    else:
        return None

def obstacle_in_front():
    """
    Check VL53L0X sensors for obstacles in front.
    Returns True if any sensor detects an object closer than threshold (cm).
    """
    dist_f = sensor_f.range / 10    # Convert mm to cm
    dist_lf = sensor_lf.range / 10
    dist_rf = sensor_rf.range / 10
    if dist_f < 25 or dist_lf < 20 or dist_rf < 20:
        return True
    return False

def obstacle_at_back_side():
    """
    Check ultrasonic sensors at back left and right.
    Returns 'left' or 'right' if obstacle detected within 15 cm, else None.
    """
    dist_lb = sensor_lb.distance * 100  # Convert m to cm
    dist_rb = sensor_rb.distance * 100
    if dist_lb < 15:
        return "left"
    elif dist_rb < 15:
        return "right"
    else:
        return None

def align_block_center(block):
    """
    Align the detected block to the vertical center line of Pixy2 FOV by adjusting rear steering servo.
    Uses proportional control based on horizontal error (block.x - center_x).
    """
    center_x = 158  # Pixy2 camera horizontal center (0-315)
    error = block.x - center_x  # Positive if block is right of center, negative if left
    Kp = 0.3                   # Proportional gain - tune for responsiveness
    angle = 90 + (error * Kp)  # Adjust servo angle from neutral (90 degrees)
    # Clamp servo angle between 45 (max left) and 135 (max right)
    angle = max(45, min(135, angle))
    set_servo_angle(angle)

def go_around_block(side):
    """
    Navigate around the block on the specified side ('left' or 'right').
    Continuously aligns the block center and moves forward until the block is passed.
    Handles obstacles by stopping and reversing if necessary.
    """
    turn_left = 45
    turn_right = 135
    straight = 90

    # Set initial steering angle to go around block on chosen side
    if side == "left":
        set_servo_angle(turn_left)
    else:
        set_servo_angle(turn_right)

    forward(30)  # Moderate forward speed
    timeout = time.time() + 10  # Max 10 seconds to avoid infinite loop

    while time.time() < timeout:
        block = detect_closest_color_block()
        if block is None or block.signature not in (GREEN_SIG, RED_SIG):
            break  # Block passed or lost

        align_block_center(block)  # Keep block centered in camera FOV

        if obstacle_in_front():
            stop()
            reverse(30)
            time.sleep(0.5)
            stop()
            break

        back_obs = obstacle_at_back_side()
        if back_obs == "left":
            set_servo_angle(turn_right)
        elif back_obs == "right":
            set_servo_angle(turn_left)

        time.sleep(0.05)  # Small delay for loop timing

    set_servo_angle(straight)  # Straighten steering after passing block
    stop()

def main_loop():
    """
    - Main control loop for the robot.
    - Detects blocks and decides navigation accordingly.
    - Moves forward safely when no blocks are detected.
    - Handles obstacle avoidance and steering adjustments.
    """
    global laps_completed
    try:
        while laps_completed < 3:
            block = detect_closest_color_block()
            if block:
                print(f"Block detected: sig {block.signature}, pos {block.x}, size {block.width}")
                side = decide_turn_side_by_signature(block)
                if side:
                    print(f"Going around block on the {side} side")
                    go_around_block(side)
                else:
                    # Unknown signature, move forward centered
                    set_servo_angle(90)
                    forward(40)
            else:
                # No block detected, move forward and avoid obstacles
                if obstacle_in_front():
                    print("Obstacle ahead! Stopping and reversing.")
                    stop()
                    reverse(30)
                    time.sleep(0.5)
                    stop()
                else:
                    set_servo_angle(90)
                    forward(40)

                # Check back obstacles and adjust steering if needed
                back_obs = obstacle_at_back_side()
                if back_obs == "left":
                    print("Back obstacle left side, steering right.")
                    set_servo_angle(135)
                elif back_obs == "right":
                    print("Back obstacle right side, steering left.")
                    set_servo_angle(45)
                else:
                    set_servo_angle(90)

            # Placeholder for lap counting logic
            # Increment laps_completed based on your own sensor or logic
            time.sleep(0.1)  # Loop delay for CPU relief and sensor update

    except KeyboardInterrupt:
        print("Stopping robot")

    finally:
        stop()
        pwm_servo.stop()
        pwm_motor.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main_loop()
