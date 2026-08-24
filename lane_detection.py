import cv2
import numpy as np
from collections import deque

class LaneDetector:
    def __init__(self):
        self.prev_error = 0
        self.left_history = deque(maxlen=10)   # store last 10 detections
        self.right_history = deque(maxlen=10)

    def region_of_interest(self, img, vertices=None):
        height, width = img.shape[:2]
        mask = np.zeros_like(img)

        if vertices is None:
            # Wider trapezoid ROI
            vertices = np.array([[
                (0, height),
                (int(width * 0.35), int(height * 0.55)),
                (int(width * 0.65), int(height * 0.55)),
                (width, height)
            ]], np.int32)

        cv2.fillPoly(mask, vertices, 255)
        return cv2.bitwise_and(img, mask)

    def color_mask(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # White mask
        white_mask = cv2.inRange(hsv, (0, 0, 200), (180, 25, 255))
        # Yellow mask
        yellow_mask = cv2.inRange(hsv, (15, 80, 80), (40, 255, 255))
        # Blue mask (optional for special lanes)
        blue_mask = cv2.inRange(hsv, (90, 80, 80), (130, 255, 255))

        combined = cv2.bitwise_or(white_mask, yellow_mask)
        combined = cv2.bitwise_or(combined, blue_mask)

        return combined

    def detect_lanes(self, frame):
        color_edges = self.color_mask(frame)

        # Combine with Canny edges for robustness
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        v = np.median(blur)
        lower = int(max(0, (1.0 - 0.33) * v))
        upper = int(min(255, (1.0 + 0.33) * v))
        edges = cv2.Canny(blur, lower, upper)

        combined_edges = cv2.bitwise_or(edges, color_edges)
        roi = self.region_of_interest(combined_edges)

        lines = cv2.HoughLinesP(
            roi, 1, np.pi / 180, threshold=50,
            minLineLength=30, maxLineGap=40
        )

        left_lines, right_lines = [], []
        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                if abs(x2 - x1) < 5:  # skip vertical
                    continue
                slope = (y2 - y1) / (x2 - x1)
                if -2.5 < slope < -0.2:
                    left_lines.append([x1, y1, x2, y2])
                elif 0.2 < slope < 2.5:
                    right_lines.append([x1, y1, x2, y2])

        left_line = self.average_line(frame, left_lines)
        right_line = self.average_line(frame, right_lines)

        # Stabilize with history
        if left_line: self.left_history.append(left_line)
        if right_line: self.right_history.append(right_line)

        left_line = self.stabilize(self.left_history)
        right_line = self.stabilize(self.right_history)

        overlay = frame.copy()
        steering, confidence = self.steering_logic(frame, left_line, right_line)
        self.visualize(overlay, frame, left_line, right_line, steering, confidence)

        return overlay, steering, self.prev_error, confidence

    def average_line(self, frame, lines):
        if len(lines) == 0:
            return None
        x_coords, y_coords = [], []
        for x1, y1, x2, y2 in lines:
            x_coords.extend([x1, x2])
            y_coords.extend([y1, y2])
        poly = np.polyfit(y_coords, x_coords, 1)
        y_max, y_min = frame.shape[0], int(frame.shape[0] * 0.6)
        x_max = int(poly[0] * y_max + poly[1])
        x_min = int(poly[0] * y_min + poly[1])
        return (x_max, y_max, x_min, y_min)

    def stabilize(self, history):
        if len(history) == 0:
            return None
        avg = np.mean(history, axis=0).astype(int)
        return tuple(avg)

    def steering_logic(self, frame, left_line, right_line):
        confidence = 0
        if left_line is not None and right_line is not None:
            lane_center = (left_line[0] + right_line[0]) // 2
            car_center = frame.shape[1] // 2
            pixel_error = lane_center - car_center
            alpha = 0.8
            self.prev_error = alpha * self.prev_error + (1 - alpha) * pixel_error
            confidence = 1.0
        elif left_line is not None:
            car_center = frame.shape[1] // 2
            pixel_error = left_line[0] - car_center * 0.75
            confidence = 0.6
        elif right_line is not None:
            car_center = frame.shape[1] // 2
            pixel_error = right_line[0] - car_center * 1.25
            confidence = 0.6
        else:
            pixel_error = self.prev_error
            confidence = 0.3

        steering_angle = np.clip(pixel_error * 0.1, -30, 30)
        if abs(steering_angle) < 3:
            steering = "STRAIGHT"
        elif steering_angle > 0:
            steering = f"RIGHT ({int(steering_angle)}°)"
        else:
            steering = f"LEFT ({int(steering_angle)}°)"
        return steering, confidence

    def visualize(self, overlay, frame, left_line, right_line, steering, confidence):
        if left_line:
            cv2.line(overlay, (left_line[0], left_line[1]),
                     (left_line[2], left_line[3]), (0, 255, 0), 8)
        if right_line:
            cv2.line(overlay, (right_line[0], right_line[1]),
                     (right_line[2], right_line[3]), (0, 0, 255), 8)

        car_center = frame.shape[1] // 2
        cv2.circle(overlay, (car_center, frame.shape[0]), 10, (255, 0, 255), -1)

        cv2.putText(overlay, f"Steering: {steering}", (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        cv2.putText(overlay, f"Confidence: {confidence:.1%}", (30, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
        cv2.putText(overlay, f"Error: {int(self.prev_error)}px", (30, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


# Usage
detector = LaneDetector()
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    result, steering, error, confidence = detector.detect_lanes(frame)
    cv2.imshow("Multi-Color Lane Detection", result)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
