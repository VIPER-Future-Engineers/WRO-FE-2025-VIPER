from pixy2 import Pixy2
import time

# Initialize Pixy2
pixy = Pixy2()

# Color signature IDs (set these according to your PixyMon config)
RED_SIG = 1
GREEN_SIG = 2

def turn_left():
    # Your motor control code for left turn
    print("Turning left")

def turn_right():
    # Your motor control code for right turn
    print("Turning right")

def stop():
    # Your motor control code to stop
    print("Stopping")

def main_loop():
    try:
        while True:
            blocks = pixy.get_blocks()
            if blocks:
                for block in blocks:
                    if block.signature == RED_SIG:
                        turn_left()
                    elif block.signature == GREEN_SIG:
                        turn_right()
                    else:
                        stop()
            else:
                stop()
            time.sleep(0.1)
    except KeyboardInterrupt:
        stop()
        print("Program stopped")

if __name__ == "__main__":
    main_loop()
