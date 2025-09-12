#!/usr/bin/env python3
import time
import cv2
import numpy as np
from picamera2 import Picamera2

class Camera:
    def __init__(self):
        # Initialize Pi Camera
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": (640, 360)})  # smaller for speed
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(2)
        print("[INFO] PiCamera2 initialized.")

    def get_frame(self):
        frame = self.picam2.capture_array()
        if frame is None:
            frame = np.zeros((360,640,3), dtype=np.uint8)
        # Convert RGB → BGR for OpenCV
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # Flip upside down
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

        # --- LESS SENSITIVE HSV THRESHOLDS ---
        # Red
        lower_red1 = np.array([0, 80, 80])
        upper_red1 = np.array([10, 200, 200])
        lower_red2 = np.array([160, 80, 80])
        upper_red2 = np.array([180, 200, 200])
        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

        # Green
        lower_green = np.array([45, 60, 60])
        upper_green = np.array([75, 200, 200])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        color_masks = [("Red", mask_red, (0,0,255)), ("Green", mask_green, (0,255,0))]

        # Draw grid
        for i in range(1, cols):
            cv2.line(frame, (i*step_x,0), (i*step_x,height), (0,255,0), 2)
        for j in range(1, rows):
            cv2.line(frame, (0,j*step_y), (width,j*step_y), (0,255,0), 2)

        # Track objects and label sections
        section_id = 1
        total_blocks = 0
        for j in range(rows):
            for i in range(cols):
                cx = i*step_x + step_x//2
                cy = j*step_y + step_y//2
                cv2.putText(frame, str(section_id), (cx-15, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0), 2)

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
                            total_blocks += 1
                            cv2.rectangle(frame, (x,y), (x+w,y+h), box_color, 2)
                            cv2.putText(frame, f"{color_name} {int(distance_est)}cm",
                                        (x, y-10), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.5, box_color, 1)
                section_id += 1

        # Display total detected blocks on frame
        cv2.putText(frame, f"Total blocks: {total_blocks}", (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

        return frame, detected_sections

def main():
    print("Starting PiCamera2 detection (3x2 grid, less sensitive). Press 'q' to exit.")
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
