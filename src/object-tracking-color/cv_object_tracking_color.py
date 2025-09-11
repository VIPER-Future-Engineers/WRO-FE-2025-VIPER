#!/usr/bin/env python3
import os, sys, cv2, time, numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils.picamera_utils import is_raspberry_camera, get_picamera

IMAGE_WIDTH, IMAGE_HEIGHT = 320, 240
IS_RASPI_CAMERA = is_raspberry_camera()

# Camera calibration constants (replace with your own calibration, if available)
# Here's an example identity-like matrix and zero distortion—replace with real values for best results
cameraMatrix = np.array([[IMAGE_WIDTH, 0, IMAGE_WIDTH/2],
                         [0, IMAGE_WIDTH, IMAGE_HEIGHT/2],
                         [0, 0, 1]], dtype=np.float32)
distCoeffs = np.zeros(5)  # assume no distortion if not calibrated

# Color ranges
GREEN_MIN = np.array([35, 50, 50])
GREEN_MAX = np.array([90, 255, 255])
RED_MIN1 = np.array([0, 70, 40])
RED_MAX1 = np.array([10, 255, 255])
RED_MIN2 = np.array([160, 70, 40])
RED_MAX2 = np.array([180, 255, 255])

APPROX_POLY_EPS = 0.04
MIN_AREA = 400
MIN_AR = 0.4

fps = 0

def undistort_frame(frame):
    return cv2.undistort(frame, cameraMatrix, distCoeffs)

def detect_rects(frame, hsv, lo, hi, label):
    mask = cv2.inRange(hsv, lo, hi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8), iterations=1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    for cnt in contours:
        if cv2.contourArea(cnt) < MIN_AREA:
            continue
        approx = cv2.approxPolyDP(cnt, APPROX_POLY_EPS * cv2.arcLength(cnt, True), True)
        if len(approx) >= 4:
            x, y, w, h = cv2.boundingRect(approx)
            if min(w,h)/max(w,h) < MIN_AR:
                continue
            cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0) if label=='green' else (0,0,255), 2)
            cv2.putText(frame, f"{label}", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                        (0,255,0) if label=='green' else (0,0,255), 1)
            found.append((x, y, w, h))
    return found

def main():
    if IS_RASPI_CAMERA:
        cap = get_picamera(IMAGE_WIDTH, IMAGE_HEIGHT)
        cap.start()
    else:
        cap = cv2.VideoCapture(0)
        cap.set(3, IMAGE_WIDTH); cap.set(4, IMAGE_HEIGHT)

    while True:
        start = time.time()
        frame = cap.capture_array() if IS_RASPI_CAMERA else cap.read()[1]
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        
        # Undistort
        frame = undistort_frame(frame)
        
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        green = detect_rects(frame, hsv, GREEN_MIN, GREEN_MAX, 'green')
        red = detect_rects(frame, hsv, RED_MIN1, RED_MAX1, 'red')
        red += detect_rects(frame, hsv, RED_MIN2, RED_MAX2, 'red')

        # Distance estimation
        g_area = max([w*h for _,_,w,h in green], default=0)
        r_area = max([w*h for _,_,w,h in red], default=0)
        status = "Green closer" if g_area>r_area else "Red closer" if r_area>g_area else "Unknown"
        cv2.putText(frame, status, (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        
        fps = 1.0 / (time.time() - start)
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, IMAGE_HEIGHT-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)

        cv2.imshow("Full FoV Detection", frame)
        if cv2.waitKey(1) & 0xFF in [27, ord('q')]:
            break

    cap.release() if not IS_RASPI_CAMERA else cap.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
