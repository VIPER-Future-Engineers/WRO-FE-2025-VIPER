from picamera2 import Picamera2
import cv2
from ultralytics import YOLO

def main():
    model = YOLO("yolo11n.pt")

    picam2 = Picamera2()
    picam2_config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(picam2_config)
    picam2.start()

    while True:
        frame = picam2.capture_array()
        if frame is None:
            print("Failed to capture frame from picamera2.")
            break

        results = model(frame)
        annotated = results[0].plot()

        cv2.imshow("YOLOv11 Detection", annotated)
        if cv2.waitKey(1) in (27, ord('q')):
            break

    picam2.stop()
    cv2.destroyAllWindows()
