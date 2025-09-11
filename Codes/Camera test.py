import cv2
import numpy as np

class CameraSensor:
    def __init__(self, resolution=(320, 240), framerate=20):
        self.cv2 = cv2
        self.resolution = resolution
        self.framerate = framerate
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        self.cap.set(cv2.CAP_PROP_FPS, framerate)

    def capture_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def _detect_rects(self, hsv, lo, hi):
        mask = cv2.inRange(hsv, lo, hi)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        found = []
        for cnt in contours:
            if cv2.contourArea(cnt) < 400:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if min(w, h) / max(w, h) < 0.4:
                continue
            found.append((x,y,w,h))
        return found

    def detect_blocks_sections(self):
        frame = self.capture_frame()
        if frame is None:
            return None

        height, width, _ = frame.shape

        sections = {
            'left_top': frame[:height // 3, :width // 2],
            'left_middle': frame[height // 3: 2 * height // 3, :width // 2],
            'left_bottom': frame[2 * height // 3:, :width // 2],
            'right_top': frame[:height // 3, width // 2:],
            'right_middle': frame[height // 3: 2 * height // 3, width // 2:],
            'right_bottom': frame[2 * height // 3:, width // 2:],
        }

        detected = {}
        for sec_name, sec_img in sections.items():
            hsv = cv2.cvtColor(sec_img, cv2.COLOR_BGR2HSV)
            green_blocks = self._detect_rects(hsv, np.array([35,50,50]), np.array([90,255,255]))
            red_blocks = self._detect_rects(hsv, np.array([0,70,40]), np.array([10,255,255]))
            red_blocks += self._detect_rects(hsv, np.array([160,70,40]), np.array([180,255,255]))
            detected[sec_name] = {'green': green_blocks, 'red': red_blocks}

        return frame, detected

def map_detected_layout_to_direction(detected_sections):
    green_in_left_middle = len(detected_sections.get('left_middle', {}).get('green', [])) > 0
    red_in_right_middle = len(detected_sections.get('right_middle', {}).get('red', [])) > 0

    if green_in_left_middle:
        return 'forward'
    elif red_in_right_middle:
        return 'forward'
    else:
        return 'turn_right'

def draw_sections_and_blocks(frame, detected_sections):
    height, width, _ = frame.shape
    # Draw section rectangles (blue)
    sections_coords = {
        'left_top': (0, 0, width // 2, height // 3),
        'left_middle': (0, height // 3, width // 2, height // 3),
        'left_bottom': (0, 2 * height // 3, width // 2, height // 3),
        'right_top': (width // 2, 0, width // 2, height // 3),
        'right_middle': (width // 2, height // 3, width // 2, height // 3),
        'right_bottom': (width // 2, 2 * height // 3, width // 2, height // 3),
    }

    for name, (x, y, w, h) in sections_coords.items():
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        green_count = len(detected_sections.get(name, {}).get('green', []))
        red_count = len(detected_sections.get(name, {}).get('red', []))
        text = f"G:{green_count} R:{red_count}"
        cv2.putText(frame, text, (x + 5, y + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        # Draw detected green blocks (green rectangles)
        for (bx, by, bw, bh) in detected_sections.get(name, {}).get('green', []):
            cv2.rectangle(frame, (x + bx, y + by), (x + bx + bw, y + by + bh), (0, 255, 0), 2)
        # Draw detected red blocks (red rectangles)
        for (bx, by, bw, bh) in detected_sections.get(name, {}).get('red', []):
            cv2.rectangle(frame, (x + bx, y + by), (x + bx + bw, y + by + bh), (0, 0, 255), 2)

def main():
    camera = CameraSensor()

    print("Starting camera detection with overlay. Press 'q' to exit.")
    try:
        while True:
            frame, detected_sections = camera.detect_blocks_sections()
            if frame is None or detected_sections is None:
                print("No frame captured, retrying...")
                continue

            direction = map_detected_layout_to_direction(detected_sections)

            draw_sections_and_blocks(frame, detected_sections)

            cv2.putText(frame, f"Action: {direction}", (10, frame.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.imshow("Camera Zones with Detection", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Stopping camera detection.")
    finally:
        camera.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
