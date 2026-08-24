import cv2
import numpy as np
from ultralytics import YOLO
from threading import Thread

class LaneDetector:

    def __init__(self):

        self.prev_error = 0
        self.frame_count = 0
        self.last_results = []

        # =========================
        # YOLO Model
        # =========================
        self.model = YOLO("yolov8n.pt")

    # =========================
    # ROI
    # =========================
    def region_of_interest(self, img):

        height, width = img.shape[:2]

        mask = np.zeros_like(img)

        polygon = np.array([[
            (0, height),
            (width, height),
            (int(width * 0.95), int(height * 0.60)),
            (int(width * 0.05), int(height * 0.60))
        ]], np.int32)

        cv2.fillPoly(mask, polygon, 255)

        return cv2.bitwise_and(img, mask)

    # =========================
    # Main Detection
    # =========================
    def detect_lanes(self, frame):

        overlay = frame.copy()

        height, width = frame.shape[:2]

        # =========================
        # OBJECT DETECTION
        # =========================
        self.frame_count += 1

        if self.frame_count % 3 == 0:
            self.last_results = self.model(
                frame,
                imgsz=320,
                verbose=False
            )

        results = self.last_results

        obstacle_detected = False
        status = "SAFE"

        for result in results:

            boxes = result.boxes

            for box in boxes:

                cls_id = int(box.cls[0])

                conf = float(box.conf[0])

                if conf < 0.4:
                    continue

                class_name = self.model.names[cls_id]

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # Draw Bounding Box
                cv2.rectangle(
                    overlay,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 255),
                    2
                )

                label = f"{class_name} {conf:.2f}"

                cv2.putText(
                    overlay,
                    label,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2
                )

                # =========================
                # Obstacle Detection
                # =========================
                object_center = (x1 + x2) // 2

                frame_center = width // 2

                if (
                    abs(object_center - frame_center) < 120
                    and y2 > height * 0.6
                ):

                    obstacle_detected = True
                    status = "STOP"

        # =========================
        # HSV Detection (Yellow Curbs)
        # =========================
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([15, 40, 80])
        upper_yellow = np.array([40, 255, 255])

        yellow_mask = cv2.inRange(
            hsv,
            lower_yellow,
            upper_yellow
        )

        # =========================
        # Grayscale + Blur
        # =========================
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # =========================
        # Edge Detection
        # =========================
        edges = cv2.Canny(blur, 40, 120)

        # =========================
        # Combine Yellow + Edges
        # =========================
        combined = cv2.bitwise_or(edges, yellow_mask)

        # =========================
        # ROI
        # =========================
        roi = self.region_of_interest(combined)

        # =========================
        # Morphology
        # =========================
        kernel = np.ones((5, 5), np.uint8)

        roi = cv2.dilate(roi, kernel, iterations=1)

        roi = cv2.erode(roi, kernel, iterations=1)

        # =========================
        # Hough Lines
        # =========================
        lines = cv2.HoughLinesP(
            roi,
            1,
            np.pi / 180,
            threshold=50,
            minLineLength=80,
            maxLineGap=60
        )

        left_lines = []
        right_lines = []

        # =========================
        # Filter Lines
        # =========================
        if lines is not None:

            for line in lines:

                x1, y1, x2, y2 = line[0]

                # Ignore small lines
                if np.hypot(x2 - x1, y2 - y1) < 40:
                    continue

                # Ignore vertical poles
                if abs(x2 - x1) < 10:
                    continue

                slope = (y2 - y1) / (x2 - x1)

                # Ignore horizontal lines
                if abs(slope) < 0.25:
                    continue

                # LEFT
                if slope < 0:
                    left_lines.append((x1, y1, x2, y2))

                # RIGHT
                else:
                    right_lines.append((x1, y1, x2, y2))

        # =========================
        # Average Lines
        # =========================
        left_line = None
        right_line = None

        if len(left_lines) > 0:

            left_avg = np.mean(left_lines, axis=0).astype(int)

            left_line = left_avg

        if len(right_lines) > 0:

            right_avg = np.mean(right_lines, axis=0).astype(int)

            right_line = right_avg

        # =========================
        # Steering
        # =========================
        steering = "STRAIGHT"

        confidence = 0.0

        error = 0

        lane_center = width // 2

        if left_line is not None and right_line is not None:

            left_x = max(left_line[0], left_line[2])

            right_x = min(right_line[0], right_line[2])

            lane_center = int((left_x + right_x) / 2)

            car_center = width // 2

            error = lane_center - car_center

            self.prev_error = error

            steering_angle = error * 0.05

            confidence = 1.0

            if abs(steering_angle) < 3:
                steering = "STRAIGHT"
                status = "SAFE"

            elif steering_angle > 0:

                steering = f"RIGHT ({int(steering_angle)}°)"
                status = "TURN RIGHT"

            else:

                steering = f"LEFT ({int(steering_angle)}°)"
                status = "TURN LEFT"

            # Draw Lane Center
            cv2.line(
                overlay,
                (lane_center, height),
                (lane_center, int(height * 0.55)),
                (255, 255, 0),
                4
            )

        # =========================
        # Visualization
        # =========================
        if obstacle_detected:
            status = "STOP"

        self.visualize(
            overlay,
            frame,
            left_line,
            right_line,
            steering,
            confidence,
            lane_center,
            obstacle_detected
        )

        return overlay, steering, error, confidence, status

    # =========================
    # Visualization
    # =========================
    def visualize(
        self,
        overlay,
        frame,
        left_line,
        right_line,
        steering,
        confidence,
        lane_center,
        obstacle_detected
    ):

        height, width = frame.shape[:2]

        # LEFT LINE
        if left_line is not None:

            cv2.line(
                overlay,
                (left_line[0], left_line[1]),
                (left_line[2], left_line[3]),
                (0, 255, 0),
                8
            )

        # RIGHT LINE
        if right_line is not None:

            cv2.line(
                overlay,
                (right_line[0], right_line[1]),
                (right_line[2], right_line[3]),
                (0, 0, 255),
                8
            )

        # CAR CENTER
        car_center = width // 2

        cv2.circle(
            overlay,
            (car_center, height),
            10,
            (255, 0, 255),
            -1
        )

        # ROAD CENTER
        cv2.circle(
            overlay,
            (lane_center, height),
            10,
            (255, 255, 0),
            -1
        )

        # CONNECT LINE
        cv2.line(
            overlay,
            (car_center, height),
            (lane_center, height),
            (255, 0, 255),
            3
        )


# =========================
# MAIN
# =========================
if __name__ == "__main__":

    class VideoCapture:

        def __init__(self, rtsp_url):

            self.cap = cv2.VideoCapture(
                rtsp_url,
                cv2.CAP_FFMPEG
            )

            self.ret, self.frame = self.cap.read()

            Thread(
                target=self.update,
                daemon=True
            ).start()

        def update(self):

            while True:

                ret, frame = self.cap.read()

                if ret:
                    self.frame = frame

        def read(self):

            return self.frame

    detector = LaneDetector()

    # Dahua RTSP Stream
    rtsp_url = (
        rtsp_url = "rtsp://USERNAME:PASSWORD@CAMERA_IP:554/cam/realmonitor?channel=1&subtype=1"
    )

    camera = VideoCapture(rtsp_url)

    while True:

        # Skip old buffered frames
        frame = camera.read()

        if frame is None:
            continue

        # Resize for faster YOLO
        frame = cv2.resize(frame, (640, 480))

        result, steering, error, confidence, status = (
            detector.detect_lanes(frame)
        )

        cv2.imshow(
            "Autonomous Road Centering + Object Detection",
            result
        )

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cv2.destroyAllWindows()
