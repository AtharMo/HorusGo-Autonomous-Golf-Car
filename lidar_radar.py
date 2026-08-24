import cv2
import numpy as np
import math


class RadarDisplay:

    def __init__(self):

        self.size = 500

        self.center_x = self.size // 2
        self.center_y = self.size // 2

        self.max_distance = 6000  # mm

        self.sweep_angle = 0

    def draw(self, scan_data):

        radar = np.zeros(
            (self.size, self.size, 3),
            dtype=np.uint8
        )

        # Background
        radar[:] = (15, 23, 42)

        # =========================
        # Radar Rings
        # =========================

        for radius in [75, 150, 225]:

            cv2.circle(
                radar,
                (self.center_x, self.center_y),
                radius,
                (0, 120, 0),
                1
            )

        # =========================
        # Cross Lines
        # =========================

        cv2.line(
            radar,
            (self.center_x, 0),
            (self.center_x, self.size),
            (0, 120, 0),
            1
        )

        cv2.line(
            radar,
            (0, self.center_y),
            (self.size, self.center_y),
            (0, 120, 0),
            1
        )

        # =========================
        # Angle Labels
        # =========================

        cv2.putText(
            radar,
            "0",
            (self.center_x - 10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            1
        )

        cv2.putText(
            radar,
            "90",
            (20, self.center_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            1
        )

        cv2.putText(
            radar,
            "180",
            (self.center_x - 20, self.size - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            1
        )

        cv2.putText(
            radar,
            "270",
            (self.size - 50, self.center_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            1
        )

        # =========================
        # Distance Labels
        # =========================

        cv2.putText(
            radar,
            "2m",
            (self.center_x + 80, self.center_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 180, 0),
            1
        )

        cv2.putText(
            radar,
            "4m",
            (self.center_x + 155, self.center_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 180, 0),
            1
        )

        cv2.putText(
            radar,
            "6m",
            (self.center_x + 230, self.center_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 180, 0),
            1
        )

        # =========================
        # Radar Sweep
        # =========================

        self.sweep_angle += 3

        if self.sweep_angle >= 360:
            self.sweep_angle = 0

        sweep_rad = math.radians(self.sweep_angle)

        sweep_x = int(
            self.center_x +
            225 * math.cos(sweep_rad)
        )

        sweep_y = int(
            self.center_y -
            225 * math.sin(sweep_rad)
        )

        cv2.line(
            radar,
            (self.center_x, self.center_y),
            (sweep_x, sweep_y),
            (0, 255, 0),
            2
        )

        # =========================
        # Golf Cart Position
        # =========================

        cv2.circle(
            radar,
            (self.center_x, self.center_y),
            12,
            (0, 255, 255),
            -1
        )

        cv2.putText(
            radar,
            "HorusGo",
            (self.center_x - 35, self.center_y + 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1
        )

        # =========================
        # Obstacles
        # =========================

        for angle, distance in scan_data:

            if distance <= 0:
                continue

            if distance > self.max_distance:
                continue

            radius = int(
                (distance / self.max_distance)
                * 225
            )

            rad = math.radians(angle)

            x = int(
                self.center_x +
                radius * math.cos(rad)
            )

            y = int(
                self.center_y -
                radius * math.sin(rad)
            )

            # Red if dangerous
            if distance < 3000:

                color = (0, 0, 255)

            else:

                color = (0, 255, 255)

            cv2.circle(
                radar,
                (x, y),
                4,
                color,
                -1
            )

        # =========================
        # Title
        # =========================

        cv2.putText(
            radar,
            "HORUSGO RADAR",
            (140, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2
        )

        return radar