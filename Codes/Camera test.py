#!/usr/bin/env python3
import time
import cv2
import numpy as np
from picamera2 import Picamera2


class Camera:
    def __init__(self):
        # Initialize Pi Camera (instead of cv2.VideoCapture)
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"size": (640, 480)})
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(2)  # warm-up time
        print("[INFO] PiCamera2 initialized.")

    def get_frame(self):
        """Capture a frame from the Pi Camera."""
        return self.picam2.capture_array()

    def detect_blocks_sections(self):
        """
        Capture a frame, overlay a 3x3 grid, detect colored blocks,
        and return (frame, detected_sections).
        """
        frame = self.get_frame()
        if frame is None:
            print("[WARN] No frame captured.")
            return np.zeros((480, 640, 3), dtype=np.uint8), []

        height, width, _ = frame.shape
        step_x = width // 3
        step_y = height // 3

        detected_sections = []

        # Convert to HSV for color-based object detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Example: detect RED objects
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = mask1 | mask2
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw grid
        for i in range(1, 3):
            cv2.line(frame, (i * step_x, 0), (i * step_x, height), (0, 255, 0), 2)
            cv2.line(frame, (0, i * step_y), (width, i * step_y), (0, 255, 0), 2)

        # Label grid + check which section detections fall into
        section_id = 1
        for y in range(3):
            for x in range(3):
                cx = x * step_x + step_x // 2
                cy = y * step_y + step_y // 2
                cv2.putText(frame, str(section_id), (cx - 10, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                # Check if any contour is inside this section
                for cnt in contours:
                    x1, y1, w, h = cv2.boundingRect(cnt)
                    obj_cx = x1 + w // 2
                    obj_cy = y1 + h // 2

                    if (x * step_x <= obj_cx < (x + 1) * step_x and
                        y * step_y <= obj_cy < (y + 1) * step_y):
                        detected_sections.append(section_id)
                        cv2.rectangle(frame, (x1, y1), (x1 + w, y1 + h), (255, 0, 0), 2)

                section_id += 1

        return frame, detected_sections


def main():
    print("Starting PiCamera2 detection with grid overlay. Press 'q' to exit.")
    camera = Camera()

    while True:
        frame, detected_sections = camera.detect_blocks_sections()
        # Just print what sections currently have detected objects
        if detected_sections:
            print(f"[INFO] Detected blocks in sections: {detected_sections}")

        cv2.imshow("Camera", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
