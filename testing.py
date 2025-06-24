from gpiozero import DistanceSensor
from time import sleep
import RPi.GPIO as GPIO
import board
import busio
import adafruit_vl53l0x

# Create I2C interface
i2c = busio.I2C(board.SCL, board.SDA)

# Create VL53L0X sensor object
vl53 = adafruit_vl53l0x.VL53L0X(i2c)
# Initialize the ultrasonic sensors
sensor_rf = DistanceSensor(echo=, trigger=)   # Right Front sensor
sensor_rb = DistanceSensor(echo=26, trigger=16)   # Right Back sensor
sensor_f = DistanceSensor(echo=, trigger=)    # Front sensor
sensor_lf = DistanceSensor(echo=, trigger=)   # Right Front sensor
sensor_lb = DistanceSensor(echo=20, trigger=21)   # Right Back sensor
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

# Initialize PWM for motor and servo control
pwm_servo = GPIO.PWM(SERVO_PIN, 50)  # Servo: 50 Hz
pwm_motor = GPIO.PWM(ENA, 100)       # Motor: 100 Hz

# Start PWM outputs
pwm_servo.start(90)  # Neutral position
pwm_motor.start(0)


if colour_count == 4:
    sleep(0.01)
    lap = lap+1
    sleep(0.01)
    colour_count == 0
    
def set_angle(angle):
    """Rotate servo to specified angle (0-180)."""
    if not 0 <= angle <= 180:
        print("Angle out of range. Must be 0-180 degrees.")
        return
    duty = 2 + (angle / 16)  # Adjust duty calculation for faster response
    pwm_servo.ChangeDutyCycle(duty)

def reverse(speed):
    """Run motor in reverse direction at specified speed (0-100)."""
    if not 0 <= speed <= 100:
        print("Speed out of range. Must be 0-100%.")
        return
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm_motor.ChangeDutyCycle(speed)

def forward(speed):
    """Run motor in forward direction at specified speed (0-100)."""
    if not 0 <= speed <= 100:
        print("Speed out of range. Must be 0-100%.")
        return
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm_motor.ChangeDutyCycle(speed)

def stop():
    """Stop the motor."""
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.HIGH)

def coast():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

def direction():

    if sensor3 > 20:
        while sensor3 >= 20:  # forward until sensor sees 20 cm or less
            forward(15)
        
    else:
        stop()
        if '''colour sensor''' == '''blue colour value''':
            orientation = 'counterclockwise'
            colour_count += 1
            print('counterclockwise')
        elif '''colour sensor''' == '''orange colour value''':
            orientation = 'clockwise'
            colour_count += 1
            print('clockwise')
        else:
            print("Reading Error")
      
def setup():
    colour_count = 0
    lap = 0
    set_angle(90)
    direction()
                 
def perform_sequence_clockwise():
    """Perform distance-based movement and steering."""
    # Get distance from front-facing sensor (Sensor 3)
    distance1 = sensor_rf.distance * 100 # Right front sensor in cm (outer)
    distance2 = sensor_rb.distance * 100 # Right back sensor in cm (outer)
    distance3 = sensor_f.distance * 100  # Front sensor in cm
    distance4 = sensor_lf.distance * 100 #Left front sensor in cm (inner)
    distance5 = sensor_lb.distance * 100 #Left back sensor in cm (inner)
        
    # Print distance for debugging
    print(f"Front Distance: {distance3:.1f} cm")

    # Maximum range for Sensor 3
    max_range = 100
    min_range = 40
    
    if distance3 == 'distance':
       # Ensure distance is within the range
        if min_range <= distance3 <= max_range:
            # Calculate steering angle proportionally (closer = sharper turn)
            angle = 90 + ((max_range + distance3) / (max_range + min_range) * 25)  # 25 is the maximum turn degree
            set_angle(max(angle, 107.5))  # Limit the left angle
            forward(20)  # Increase forward speed for faster response
        
        elif distance3 < min_range:
            # Too close to the wall
            set_angle(107.5)  # Sharper turn
            forward(20)
            
        sleep(0.0001)  # Reduced delay for quicker loop iteration
    else:
        set_angle(90)
        forward(15)
    
def perform_sequence_counterclockwise():
    """Perform distance-based movement and steering."""
    # Get distance from front-facing sensor (Sensor 3)
    distance1 = sensor_rf.distance * 100 # Right front sensor in cm (inner)
    distance2 = sensor_rb.distance * 100 # Right back sensor in cm (inner)
    distance3 = sensor_f.distance * 100  # Front sensor in cm
    distance4 = sensor_lf.distance * 100 #Left front sensor in cm (outer)
    distance5 = sensor_lb.distance * 100 #Left back sensor in cm (outer)

    # Print distance for debugging
    print(f"Front Distance: {distance3:.1f} cm")

    # Maximum range for Sensor 3
    max_range = 100
    min_range = 40

    # Ensure distance is within the range
    if min_range <= distance3 <= max_range:
        # Calculate steering angle proportionally (closer = sharper turn)
        angle = 90 - ((max_range - distance3) / (max_range - min_range) * 25)  # 25 is the maximum turn degree
        set_angle(max(angle, 65))  # Limit the left angle
        forward(20)  # Increase forward speed for faster response
    elif distance3 < min_range:
        # Too close to the wall
        set_angle(64.5)  # Sharper turn
        forward(20)
    else:
        # Default behavior when out of range
        set_angle(90)  # Straight
        forward(20)

    sleep(0.0001)  # Reduced delay for quicker loop iteration


try:
    setup()
    if orientation == 'clockwise':
        while True:
            perform_sequence_clockwise()
    elif orientation == 'counterclockwise':
        while True:
            perform_sequence_counterclockwise()
        

except laps == 3:
    print("Laps complete")


finally:
    pwm_servo.stop()
    pwm_motor.stop()
    GPIO.cleanup()
