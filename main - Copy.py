import sys
import cv2
from threading import Thread

from PySide6.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QGridLayout,
    QSizePolicy,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLineEdit
)

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap

from Horus_Lidar import HorusLidar
from lidar_radar import RadarDisplay
from combined_system import LaneDetector

from import_pandas_as_pd import HorusLiveViewer

gps_viewer = HorusLiveViewer()

import serial

ultra_serial = serial.Serial(
    "COM7",
    115200,
    timeout=1
)

front_distance = 0
left_distance = 0
right_distance = 0

def read_ultrasonic():
    global front_distance
    global left_distance
    global right_distance

    try:
        line = ultra_serial.readline().decode().strip()

        parts = line.split(",")

        front_distance = int(parts[0].split(":")[1])
        left_distance = int(parts[1].split(":")[1])
        right_distance = int(parts[2].split(":")[1])

    except:
        pass


# =========================
# Init System
# =========================

lidar_sensor = HorusLidar()
lidar_sensor.start()
radar_display = RadarDisplay()
detector = LaneDetector()


# =========================
# App Setup
# =========================
app = QApplication(sys.argv)

window = QWidget()
window.setWindowTitle("HorusGo")
window.setStyleSheet("""
    QWidget {
        background-color: #8B6F00;
    }
""")
window.showFullScreen()

main_layout = QVBoxLayout()
layout = QGridLayout()


# =========================
# Camera Panel
# =========================
camera_frame = QLabel()
camera_frame.setAlignment(Qt.AlignCenter)
camera_frame.setMinimumSize(0, 0)
camera_frame.setStyleSheet("""
    border:2px solid #334155;
    border-radius:15px;
    background-color:black;
""")


# =========================
# LiDAR Panel
# =========================
lidar_frame = QLabel()
lidar_frame.setMinimumSize(0,0)
lidar_frame.setAlignment(Qt.AlignCenter)
lidar_frame.setStyleSheet("""
    border:2px solid #334155;
    border-radius:15px;
    background-color:#111827;
    color:white;
    font-size:20px;
""")


# =========================
# Ultrasonic Panel
# =========================
def update_ultrasonic_panel():

    ultrasonic_frame.setText(
        f"ULTRASONIC DATA\n\n"
        f"Front : {front_distance} cm\n"
        f"Left  : {left_distance} cm\n"
        f"Right : {right_distance} cm"
    )

ultrasonic_frame = QLabel(
    "ULTRASONIC DATA\n\n"
    "Front : 35 cm\n"
    "Left : 60 cm\n"
    "Right : 42 cm"
)
ultrasonic_frame.setAlignment(Qt.AlignTop)
ultrasonic_frame.setStyleSheet("""
    border:2px solid #334155;
    border-radius:15px;
    background-color:#111827;
    color:white;
    font-size:18px;
    padding:15px;
""")


# =========================
# Warning Panel
# =========================
warning_frame = QLabel("WELCOME TO HORUSGO\n\nSystem Ready")
warning_frame.setAlignment(Qt.AlignCenter)
warning_frame.setStyleSheet("""
    background-color:#16A34A;
    color:white;
    border-radius:15px;
    font-size:22px;
    font-weight:bold;
""")


# =========================
# GPS Panel
# =========================
gps_frame = QLabel("GPS CAMPUS MAP")
gps_frame.setAlignment(Qt.AlignCenter)
gps_frame.setStyleSheet("""
    border:2px solid #334155;
    border-radius:15px;
    background-color:#111827;
    color:white;
    font-size:20px;
    font-weight:bold;
""")

# =========================
# Control Buttons
# =========================

button_layout = QHBoxLayout()

manual_btn = QPushButton("MANUAL MODE")
auto_btn = QPushButton("AUTONOMOUS MODE")
start_btn = QPushButton("START RIDE")

start_btn.setEnabled(False)

for btn in [manual_btn, auto_btn, start_btn]:
    btn.setFixedHeight(60)
    btn.setStyleSheet("""
        QPushButton{
            background-color:#1F2937;
            color:white;
            font-size:18px;
            font-weight:bold;
            border-radius:12px;
        }

        QPushButton:hover{
            background-color:#374151;
        }
    """)

# =========================
# Route Inputs
# =========================

from_input = QLineEdit()
from_input.setPlaceholderText("From (e.g. Gate 1)")

to_input = QLineEdit()
to_input.setPlaceholderText("To (e.g. Engineering Faculty)")

for box in [from_input, to_input]:
    box.setFixedHeight(50)
    box.setStyleSheet("""
        QLineEdit{
            background-color:white;
            color:black;
            font-size:16px;
            border-radius:10px;
            padding-left:10px;
        }
    """)

button_layout.addWidget(manual_btn)
button_layout.addWidget(auto_btn)
button_layout.addWidget(start_btn)
button_layout.addWidget(from_input)
button_layout.addWidget(to_input)

# =========================
# University Logo
# =========================
logo_label = QLabel(window)

logo_pixmap = QPixmap("hue.jpg")

logo_label.setPixmap(
    logo_pixmap.scaled(
        180,
        90,
        Qt.KeepAspectRatio,
        Qt.SmoothTransformation
    )
)

logo_label.resize(180, 90)

logo_label.raise_()
logo_label.show()

print("Window size:", window.width(), window.height())
print("Logo position:", logo_label.pos())

# =========================
# Layout
# =========================
layout.addWidget(camera_frame, 0, 0)
layout.addWidget(lidar_frame, 0, 1)

# Row 1 (left + middle + big GPS spanning both rows)
layout.addWidget(warning_frame, 1, 0)
layout.addWidget(ultrasonic_frame, 1, 1)
layout.addWidget(gps_frame, 0, 2, 2, 1)  # <-- KEY: spans 2 rows

layout.setColumnStretch(0, 3)  # Camera / Warning
layout.setColumnStretch(1, 3)  # LiDAR / Ultrasonic
layout.setColumnStretch(2, 5)  # GPS (BIGGEST)

layout.setRowStretch(0, 3)  # Camera + LiDAR row
layout.setRowStretch(1, 2)  # Warning + Ultrasonic row

main_layout.addLayout(layout)
main_layout.addLayout(button_layout)

window.setLayout(main_layout)


# =========================
# Video Capture Thread
# =========================
class VideoCapture:
    def __init__(self, rtsp_url):
        self.cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        self.frame = None

        Thread(target=self.update, daemon=True).start()

    def update(self):
        while True:
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame

    def read(self):
        return self.frame


rtsp_url = "rtsp://admin:***********@192.168.1.108:554/cam/realmonitor?channel=1&subtype=1"
camera = VideoCapture(rtsp_url)


# =========================
# Warning Logic
# =========================
def update_warning(status):
    if status == "SAFE":
        warning_frame.setStyleSheet("background-color:#16A34A; color:white; border-radius:15px; font-size:22px; font-weight:bold;")
        warning_frame.setText(f"WELCOME TO HORUSGO\n\n✓ SAFE")

    elif status == "STOP":
        warning_frame.setStyleSheet("background-color:#DC2626; color:white; border-radius:15px; font-size:22px; font-weight:bold;")
        warning_frame.setText(f"WELCOME TO HORUSGO\n\n🚨 STOP\n\nObstacle Ahead")

    elif status == "TURN LEFT":
        warning_frame.setStyleSheet("background-color:#F59E0B; color:black; border-radius:15px; font-size:22px; font-weight:bold;")
        warning_frame.setText(f"WELCOME TO HORUSGO\n\n⬅ TURN LEFT")

    elif status == "TURN RIGHT":
        warning_frame.setStyleSheet("background-color:#F59E0B; color:black; border-radius:15px; font-size:22px; font-weight:bold;")
        warning_frame.setText(f"WELCOME TO HORUSGO\n\n➡ TURN RIGHT")


# =========================
# LiDAR Update
# =========================
def update_lidar():
    if not hasattr(lidar_sensor, "scan_data"):
        return
    if lidar_sensor is None:
        return

    radar_img = radar_display.draw(lidar_sensor.scan_data)

    rgb = cv2.cvtColor(radar_img, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb.shape

    image = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
    lidar_frame.setPixmap(QPixmap.fromImage(image))


# =========================
# Camera Update
# =========================
def update_frame():
    update_lidar()

    frame = camera.read()
    if frame is None:
        return

    frame = cv2.resize(frame, (640, 480))

    result, steering, error, confidence, status = detector.detect_lanes(frame)

    update_warning(status)

    rgb = cv2.cvtColor(result, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb.shape

    image = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)

    camera_frame.setPixmap(
        QPixmap.fromImage(image).scaled(
            camera_frame.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
    )


# =========================
# GPS Map
# =========================
gps_pixmap = gps_viewer.get_qimage()
gps_frame.setPixmap(
    gps_pixmap.scaled(
        gps_frame.size(),
        Qt.KeepAspectRatio,
        Qt.SmoothTransformation
    )
)

logo_label.setStyleSheet("background: transparent;")
logo_label.adjustSize()

# =========================
# Button Logic
# =========================

current_mode = "MANUAL"


def set_manual():
    global current_mode

    current_mode = "MANUAL"
    start_btn.setEnabled(False)

    route_label.setText("")

    manual_btn.setStyleSheet("""
        QPushButton{
            background-color:#16A34A;
            color:white;
            font-size:18px;
            font-weight:bold;
            border-radius:12px;
        }
    """)


def set_auto():
    global current_mode

    current_mode = "AUTO"
    start_btn.setEnabled(True)

    auto_btn.setStyleSheet("""
        QPushButton{
            background-color:#2563EB;
            color:white;
            font-size:18px;
            font-weight:bold;
            border-radius:12px;
        }
    """)


def start_ride():

    start_point = from_input.text()
    end_point = to_input.text()

    if not start_point or not end_point:
        warning_frame.setText(
            "⚠ PLEASE ENTER\nFROM AND TO"
        )
        return

    print(f"Starting ride from {start_point} to {end_point}")

    warning_frame.setText(
        f"AUTONOMOUS MODE\n\nFROM: {start_point}\nTO: {end_point}"
    )


manual_btn.clicked.connect(set_manual)
auto_btn.clicked.connect(set_auto)
start_btn.clicked.connect(start_ride)

# =========================
# Timer
# =========================
timer = QTimer()
timer.timeout.connect(update_frame)
timer.start(100)



# =========================
# Run App
# =========================
window.show()

logo_label.move(
    window.width() - logo_label.width() - 20,
    20
)

logo_label.raise_()
logo_label.show()
sys.exit(app.exec())
