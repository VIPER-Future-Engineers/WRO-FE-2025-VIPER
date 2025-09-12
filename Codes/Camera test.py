#!/usr/bin/env python3
import time
import cv2
import numpy as np
from picamera2 import Picamera2

class Camera:
    def __init__(self):
        # Initialize Pi Camera
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": (1280, 720)})
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(2)
        print("[INFO] PiCamera2 initialized.")

    def get_frame(self):
        frame = self.picam2.capture_array()
        if frame is None:
            frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        # Convert RGB (from Picamera2) to BGR for OpenCV
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # Flip for upside-down mounting
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        return frame

    def detect_blocks_sections(self):
        frame = self.get_frame()
        height, width, _ = frame.shape

        cols, rows = 2, 3  # 2 columns, 3 rows
        step_x = width // cols
        step_y = height // rows

        detected_sections = []

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red detection
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

        # Green detection
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        color_masks = [("Red", mask_red, (0,0,255)), ("Green", mask_green, (0,255,0))]

        # Draw 3x2 grid borders
        for i in range(1, cols):
            cv2.line(frame, (i*step_x, 0), (i*step_x, height), (0,255,0), 3)
        for j in range(1, rows):
            cv2.line(frame, (0, j*step_y), (width, j*step_y), (0,255,0), 3)

        # Label sections and track objects
        section_id = 1
        for j in range(rows):
            for i in range(cols):
                cx = i*step_x + step_x//2
                cy = j*step_y + step_y//2
                cv2.putText(frame, str(section_id), (cx-15, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 2)

                for color_name, mask, box_color in color_masks:
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    for cnt in contours:
                        x, y, w, h = cv2.boundingRect(cnt)
                        obj_cx = x + w//2
                        obj_cy = y + h//2
                        distance_est = 1000 / (w + 1)

                        if (i*step_x <= obj_cx < (i+1)*step_x and
                            j*step_y <= obj_cy < (j+1)*step_y):
                            detected_sections.append((section_id, color_name, distance_est))
                            cv2.rectangle(frame, (x,y), (x+w,y+h), box_color, 2)
                            cv2.putText(frame, f"{color_name} {int(distance_est)}cm",
                                        (x, y-10), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.6, box_color, 2)

                section_id += 1

        return frame, detected_sections

def main():
    print("Starting PiCamera2 detection (3x2 grid, upside-down). Press 'q' to exit.")
    camera = Camera()

    while True:
        frame, detected_sections = camera.detect_blocks_sections()
        if detected_sections:
            print(f"[INFO] Detected blocks: {detected_sections}")

        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
