from rplidar import RPLidar, RPLidarException
import time
import threading
import winsound
import pyttsx3
import os

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame


class BackgroundMusic:
    """Handles looping MP3 background music for the entire journey."""

    @staticmethod
    def play():
        try:
            pygame.mixer.init()
            script_dir = os.path.dirname(os.path.abspath(__file__))
            mp3_files = [f for f in os.listdir(script_dir) if f.endswith('.mp3')]

            if not mp3_files:
                print(f"⚠️ Could not find any MP3 files inside the folder: {script_dir}")
                return

            full_file_path = os.path.join(script_dir, mp3_files[0])
            pygame.mixer.music.load(full_file_path)
            pygame.mixer.music.set_volume(0.3)
            pygame.mixer.music.play(-1)  # -1 means loop infinitely
        except Exception as e:
            print(f"Music Engine Error: {e}")

    @staticmethod
    def stop():
        """Stops the music entirely (used when shutting down the cart)."""
        try:
            if pygame.mixer.get_init():
                pygame.mixer.music.stop()
        except:
            pass


class VoiceAssistant:
    """Handles background text-to-speech announcements."""

    @staticmethod
    def speak(text):
        def _say():
            try:
                engine = pyttsx3.init()
                engine.setProperty('rate', 175)
                voices = engine.getProperty('voices')
                for voice in voices:
                    if "Zira" in voice.name or "female" in voice.languages:
                        engine.setProperty('voice', voice.id)
                        break
                engine.say(text)
                engine.runAndWait()
            except Exception as e:
                print(f"Voice Assistant Error: {e}")

        speech_thread = threading.Thread(target=_say, daemon=True)
        speech_thread.start()


class HorusLidar:
    """
    A threaded, self-healing 360° LiDAR safety system.
    Runs continuously in the background, triggering alerts on proximity breaches.
    """

    def __init__(self, port='COM3', danger_distance_mm=3000):
        self.scan_data = []
        self.port = port
        self.danger_distance_mm = danger_distance_mm
        self.beep_freq = 1500
        self.beep_duration = 150

        self.lidar = None
        self.latest_scan = []
        self.running = False

        self.obstacle_detected = False
        self.is_moving = False
        self.previous_map = {}
        self.motion_confidence = 0

        self._lock = threading.Lock()
        self.thread = None

    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            print(f"Initializing 360° LiDAR Safety System on {self.port}...")

    def _run(self):
        while self.running:
            try:
                self.lidar = RPLidar(self.port)
                self.lidar.start_motor()
                time.sleep(2)

                self.lidar._serial.reset_input_buffer()
                print("--- 360° RADAR ACTIVE ---")

                for scan in self.lidar.iter_scans():
                    if not self.running:
                        break
                    with self._lock:
                        self.latest_scan = scan
                        self.scan_data = scan
                    self._process_scan(scan)

            except RPLidarException as e:
                print(f"⚠️ Connection glitch detected ({e}). Healing pipe...")
                self._cleanup()
                time.sleep(1.5)
                print("Reconnecting to LiDAR...")
            except Exception as e:
                self._cleanup()
                time.sleep(1)

    def _process_scan(self, scan):
        # 1. SAFETY SHIELD CALCULATION
        breach = False
        closest_angle = None
        closest_distance = None
        current_map = {}

        for (_, angle, distance) in scan:
            if distance > 0:
                current_map[int(angle)] = distance

            if 0 < distance < self.danger_distance_mm:
                breach = True
                closest_angle = angle
                closest_distance = distance

        self.obstacle_detected = breach

        if breach:
            dist_m = closest_distance / 1000.0
            print(
                f"⚠️ 360° BREACH! Distance: {dist_m:.2f}m ({closest_distance:.0f}mm) | Angle: {closest_angle:.1f}° -> BRAKING!")
            winsound.Beep(self.beep_freq, self.beep_duration)

        # 2. EGO-MOTION DETECTION
        if self.previous_map:
            delta_sum = 0
            points_compared = 0

            for angle, dist in current_map.items():
                if angle in self.previous_map:
                    delta_sum += abs(dist - self.previous_map[angle])
                    points_compared += 1

            if points_compared > 30:
                avg_shift_mm = delta_sum / points_compared

                if avg_shift_mm > 40.0:
                    self.motion_confidence = min(self.motion_confidence + 2, 10)
                else:
                    self.motion_confidence = max(self.motion_confidence - 1, 0)

                if self.motion_confidence >= 6:
                    self.is_moving = True
                elif self.motion_confidence == 0:
                    self.is_moving = False

        self.previous_map = current_map

    def _cleanup(self):
        try:
            if self.lidar:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
        except:
            pass

    def stop(self):
        self.running = False
        self._cleanup()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)


# =====================================================================
# --- MAIN CONTROL LOOP ---
# =====================================================================
if __name__ == '__main__':
    # ⚠️ IMPORTANT: Update 'COM3' to your actual physical USB port
    radar = HorusLidar(port='COM3', danger_distance_mm=3000)
    radar.start()

    # State tracking for the mobile app journey
    journey_started = False

    try:
        print("\nHorusGo Engine Ready. Waiting for journey to begin...")

        while True:
            # 1. JOURNEY INITIATION
            # This triggers the very first time the cart rolls forward.
            # Later, you can also link this to a "Start Trip" button from your mobile app.
            if radar.is_moving and not journey_started:
                journey_started = True
                print("\n🚀 [JOURNEY STARTED]")

                print("\033[92m🎙️ Assistant: Welcome to HorusGO!, wishing you a happy Journey.\033[0m")
                VoiceAssistant.speak("Welcome to HorusGO!, wishing you a happy Journey.")

                print("🎵 Background Ambiance: Playing continuously...")
                BackgroundMusic.play()

            # 2. CONTINUOUS SAFETY SHIELD
            # The radar constantly monitors the 360 zone whether moving or stopped.
            if radar.obstacle_detected:
                # ⚠️ Insert emergency physical braking relay commands here
                pass

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n🛑 [JOURNEY ENDED] Manual Shutdown Initiated...")
        BackgroundMusic.stop()
        radar.stop()
        print("System safely shut down.")