#!/usr/bin/python3

import os
import sys
import cv2
import time
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.picamera_utils import is_raspberry_camera, get_picamera

CAMERA_DEVICE_ID = 0
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
IS_RASPI_CAMERA = is_raspberry_camera()
fps = 0

print("Using raspi camera: ", IS_RASPI_CAMERA)

def visualize_fps(image, fps: int):
    text_color = (0, 255, 0) if len(image.shape) == 3 else (255, 255, 255)
    cv2.putText(image, f'FPS = {fps:.1f}', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)
    return image

def angle_between_points(p1, p2, p3):
    # Calculate angle at p2 formed by p1-p2-p3
    a = np.array(p1) - np.array(p2)
    b = np.array(p3) - np.array(p2)
    cosine_angle = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-10)
    angle = np.arccos(np.clip(cosine_angle, -1.0, 1.0))
    return np.degrees(angle)

def is_cube_face(contour):
    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)

    if len(approx) < 4 or len(approx) > 6:
        return False, approx

    if not cv2.isContourConvex(approx):
        return False, approx

    area = cv2.contourArea(approx)
    if area < 1000:  # minimum size threshold
        return False, approx

    # Check angles for near-rectangle shape
    angles = []
    pts = approx.reshape(-1, 2)
    for i in range(len(pts)):
        p1 = pts[i - 1]
        p2 = pts[i]
        p3 = pts[(i + 1) % len(pts)]
        ang = angle_between_points(p1, p2, p3)
        angles.append(ang)

    # For rectangle-like shapes, angles ~ 80-100 degrees (allow some leeway)
    for ang in angles:
        if ang < 70 or ang > 110:
            # Could be skewed, but we tolerate some deviation
            # For cube tower, be flexible, so skip strict filtering here
            pass

    # Aspect ratio check on bounding box
    x, y, w, h = cv2.boundingRect(approx)
    aspect_ratio = float(w) / h if h != 0 else 0
    if aspect_ratio < 0.3 or aspect_ratio > 3.5:
        return False, approx

    # Solidity (area / convex hull area)
    hull = cv2.convexHull(approx)
    hull_area = cv2.contourArea(hull)
    if hull_area == 0:
        return False, approx
    solidity = float(area) / hull_area
    if solidity < 0.8:
        return False, approx

    return True, approx

if __name__ == "__main__":
    try:
        if IS_RASPI_CAMERA:
            cap = get_picamera(IMAGE_WIDTH, IMAGE_HEIGHT)
            cap.start()
        else:
            cap = cv2.VideoCapture(CAMERA_DEVICE_ID)
            cap.set(3, IMAGE_WIDTH)
            cap.set(4, IMAGE_HEIGHT)

        while True:
            start_time = time.time()

            if IS_RASPI_CAMERA:
                frame = cap.capture_array()
            else:
                ret, frame = cap.read()
                if not ret:
                    print("Failed to grab frame")
                    break

            frame = cv2.rotate(frame, cv2.ROTATE_180)
            blurred = cv2.GaussianBlur(frame, (3, 3), 0)
            gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)

            edges = cv2.Canny(gray, 50, 150)

            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            output = frame.copy()
            rect_count = 0

            for cnt in contours:
                valid, approx = is_cube_face(cnt)
                if valid:
                    cv2.drawContours(output, [approx], -1, (0, 255, 0), 3)
                    M = cv2.moments(approx)
                    if M['m00'] != 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        cv2.circle(output, (cx, cy), 5, (0, 0, 255), -1)
                        cv2.putText(output, f"Pts: {len(approx)}", (cx - 30, cy - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    rect_count += 1

            cv2.putText(output, f"Cubes Faces Detected: {rect_count}", (10, IMAGE_HEIGHT - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.imshow('Cube Tower Faces', visualize_fps(output, fps))
            cv2.imshow('Edges', edges)

            end_time = time.time()
            seconds = end_time - start_time
            fps = 1.0 / seconds

            print(f"FPS: {fps:.1f}, Faces detected: {rect_count}")

            if cv2.waitKey(33) == 27:
                break

    except Exception as e:
        print(e)

    finally:
        cv2.destroyAllWindows()
        if IS_RASPI_CAMERA:
            cap.close()
        else:
            cap.release()
