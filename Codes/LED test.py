import RPi.GPIO as GPIO
import time
import threading

# Pin setup
C_PIN = 22
NC_PIN = 24
NO_PIN = 23
BUTTON_LED_PIN = 27     # AL2 Halso Switch LED
TCS_LED_PIN = 17       # TCS34725 LED

GPIO.setmode(GPIO.BCM)
GPIO.setup(C_PIN, GPIO.OUT)
GPIO.setup(NC_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(NO_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(BUTTON_LED_PIN, GPIO.OUT)
GPIO.setup(TCS_LED_PIN, GPIO.OUT)

GPIO.output(C_PIN, GPIO.HIGH)

button_led = GPIO.PWM(BUTTON_LED_PIN, 100)
tcs_led = GPIO.PWM(TCS_LED_PIN, 100)

button_led.start(0)
tcs_led.start(0)

# Shared state
heartbeat_stage = 0
interrupted = False

def set_button_led(duty):
    button_led.ChangeDutyCycle(duty)

def wake_sequence():
    print("Touch awakens the machine.")
    set_button_led(100)

    print("Eyes begin to open...")
    for duty in range(0, 101, 5):
        tcs_led.ChangeDutyCycle(duty)
        time.sleep(0.04)

    print("Blink... a memory stirs.")
    for _ in range(3):
        tcs_led.ChangeDutyCycle(0)
        time.sleep(0.15)
        tcs_led.ChangeDutyCycle(100)
        time.sleep(0.15)

    print("Eyes now fully open. The machine remembers.")
    time.sleep(1)

def heartbeat_thread(mode="idle"):
    global heartbeat_stage, interrupted
    print(f"Heartbeat mode: {mode}")
    interrupted = False

    if mode == "active":
        strength = 1.0
        speed = 0.7
        cycles = 12
    else:
        strength = 0.6
        speed = 1.2
        cycles = 8

    for i in range(cycles):
        if interrupted:
            print(f"Interrupted at heartbeat stage {heartbeat_stage}")
            fading_heartbeat(start_stage=heartbeat_stage, strength=strength, speed=speed)
            return

        heartbeat_stage = i
        set_button_led(100 * strength)
        time.sleep(0.1 * speed)
        set_button_led(30 * strength)
        time.sleep(0.1 * speed)
        set_button_led(80 * strength)
        time.sleep(0.08 * speed)
        set_button_led(20 * strength)
        time.sleep(0.1 * speed)
        set_button_led(0)
        time.sleep(0.6 * speed)

    fading_heartbeat(start_stage=cycles, strength=strength, speed=speed)

def fading_heartbeat(start_stage=0, strength=1.0, speed=1.0):
    print("The pulse slows... life recedes.")
    for i in range(start_stage, 10):
        decay = 1.0 - (i * 0.1)
        delay = 1.0 + (i * 0.1)

        set_button_led(100 * strength * decay)
        time.sleep(0.1 * speed * delay)
        set_button_led(30 * strength * decay)
        time.sleep(0.1 * speed * delay)
        set_button_led(80 * strength * decay)
        time.sleep(0.08 * speed * delay)
        set_button_led(20 * strength * decay)
        time.sleep(0.1 * speed * delay)
        set_button_led(0)
        time.sleep(0.6 * speed * delay)

    print("The eyes resist, then close...")
    for duty in [40, 100, 30, 90]:
        tcs_led.ChangeDutyCycle(duty)
        time.sleep(0.12)

    for duty in [70, 50, 60, 40, 30, 20, 10, 0]:
        tcs_led.ChangeDutyCycle(duty)
        time.sleep(0.15)

    print("The gaze fades into memory.")
    tcs_led.ChangeDutyCycle(0)

def soft_reset():
    print("🔄 Soft reset.")
    set_button_led(0)
    tcs_led.ChangeDutyCycle(0)
    return False

try:
    print("Tap to toggle. Hold for 3s to reset.")
    last_state = GPIO.LOW
    press_start = None
    running = False
    heartbeat_thread_obj = None

    while True:
        current_state = GPIO.input(NO_PIN)

        if current_state == GPIO.HIGH and last_state == GPIO.LOW:
            press_start = time.time()

        if current_state == GPIO.LOW and last_state == GPIO.HIGH:
            press_duration = time.time() - press_start if press_start else 0
            if press_duration >= 3:
                running = soft_reset()
            else:
                if not running:
                    running = True
                    wake_sequence()
                    heartbeat_thread_obj = threading.Thread(target=heartbeat_thread, args=("active",))
                    heartbeat_thread_obj.start()
                else:
                    interrupted = True
                    running = False

        last_state = current_state
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Exiting gracefully.")
finally:
    button_led.stop()
    tcs_led.stop()
    GPIO.cleanup()
