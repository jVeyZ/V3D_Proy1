"""Gesture tracker with MQTT publishing for robot control.

Publishes detected gestures to broker.hivemq.com.
"""

import argparse
import json
import threading
import time
from pathlib import Path

import cv2
import mediapipe as mp
import paho.mqtt.client as mqtt


BaseOptions = mp.tasks.BaseOptions
GestureRecognizer = mp.tasks.vision.GestureRecognizer
GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
GestureRecognizerResult = mp.tasks.vision.GestureRecognizerResult
VisionRunningMode = mp.tasks.vision.RunningMode

mp_drawing = mp.tasks.vision.drawing_utils
mp_styles = mp.tasks.vision.drawing_styles
mp_hands = mp.tasks.vision.HandLandmarksConnections

MIN_SCORE_REQUIRED = 0.60
COMMAND_MAP = {
    ("right", "Pointing_Up"): "forward_right",
    ("right", "Victory"): "backward_right",
    ("left", "Pointing_Up"): "forward_left",
    ("left", "Victory"): "backward_left",
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Gesture tracker + MQTT publisher (HiveMQ)")
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera index (default: 0)")
    parser.add_argument(
        "--model",
        type=str,
        default="assets/gesture_recognizer.task",
        help="Path to MediaPipe gesture model (.task)",
    )
    parser.add_argument(
        "--broker",
        type=str,
        default="broker.hivemq.com",
        help="MQTT broker hostname",
    )
    parser.add_argument("--port", type=int, default=1883,
                        help="MQTT broker port")
    parser.add_argument(
        "--topic",
        type=str,
        default="v3d/robot/gesture",
        help="MQTT topic for gesture payloads",
    )
    parser.add_argument(
        "--min-score",
        type=float,
        default=MIN_SCORE_REQUIRED,
        help="Minimum score to publish a gesture (effective floor is 0.60)",
    )
    parser.add_argument(
        "--publish-period",
        type=float,
        default=0.30,
        help="Minimum seconds between repeated publish of same gesture",
    )
    return parser.parse_args()


class GestureMqttPublisher:
    def __init__(self, broker, port, topic):
        self.broker = broker
        self.port = int(port)
        self.topic = topic

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect

        self.connected = False

    def connect(self):
        print(f"[MQTT] Connecting to {self.broker}:{self.port} ...")
        self.client.connect_async(self.broker, self.port, keepalive=30)
        self.client.loop_start()

    def stop(self):
        try:
            self.client.loop_stop()
            self.client.disconnect()
        except Exception:
            pass

    def publish_json(self, payload):
        if not self.connected:
            return
        self.client.publish(self.topic, json.dumps(payload), qos=0, retain=False)

    def _on_connect(self, _client, _userdata, _flags, reason_code, _properties):
        self.connected = (reason_code == 0)
        if self.connected:
            print("[MQTT] Connected")
        else:
            print(f"[MQTT] Connection failed: {reason_code}")

    def _on_disconnect(self, _client, _userdata, _flags, reason_code, _properties):
        self.connected = False
        if reason_code != 0:
            print(f"[MQTT] Disconnected unexpectedly: {reason_code}")


def draw_result(frame_bgr, result):
    if not result or not result.hand_landmarks:
        return frame_bgr

    h, w, _ = frame_bgr.shape
    out = frame_bgr.copy()

    for i, landmarks in enumerate(result.hand_landmarks):
        mp_drawing.draw_landmarks(
            out,
            landmarks,
            mp_hands.HAND_CONNECTIONS,
            mp_styles.get_default_hand_landmarks_style(),
            mp_styles.get_default_hand_connections_style(),
        )

        if result.gestures and i < len(result.gestures) and result.gestures[i]:
            gesture = result.gestures[i][0]
            label = f"{gesture.category_name}  {gesture.score:.0%}"
            xs = [lm.x for lm in landmarks]
            ys = [lm.y for lm in landmarks]
            tx = max(0, int(min(xs) * w))
            ty = max(20, int(min(ys) * h) - 15)
            cv2.putText(
                out,
                label,
                (tx, ty),
                cv2.FONT_HERSHEY_DUPLEX,
                0.8,
                (88, 205, 54),
                2,
                cv2.LINE_AA,
            )

    return out


def get_hand_label(result, hand_index):
    if (
        result is None
        or not getattr(result, "handedness", None)
        or hand_index >= len(result.handedness)
        or not result.handedness[hand_index]
    ):
        return None
    return str(result.handedness[hand_index][0].category_name).lower()


def extract_commands(result, threshold):
    """Extracts command messages from recognized gestures by hand side."""
    if result is None or not getattr(result, "gestures", None):
        return []

    commands = []
    for i, hand_gestures in enumerate(result.gestures):
        if not hand_gestures:
            continue

        g = hand_gestures[0]
        if g.score < threshold:
            continue

        hand = get_hand_label(result, i)
        if hand is None:
            continue

        command = COMMAND_MAP.get((hand, g.category_name))
        if command is None:
            continue

        commands.append(
            {
                "command": command,
                #"gesture": g.category_name,
                #"hand": hand,
                #"score": round(float(g.score), 4),
                "timestamp_ms": int(time.time() * 1000),
            }
        )

    return commands


def main():
    args = parse_args()

    model_path = Path(args.model)
    if not model_path.exists():
        print(f"[ERROR] Model file not found: {model_path}")
        return

    cam = cv2.VideoCapture(args.camera)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cam.isOpened():
        print(f"[ERROR] Could not open camera index {args.camera}")
        return

    latest_result = None
    result_lock = threading.Lock()
    start_time = time.monotonic()

    mqtt_pub = GestureMqttPublisher(args.broker, args.port, args.topic)
    mqtt_pub.connect()

    last_publish_ts_by_command = {}

    def on_result(result, _output_image, _timestamp_ms):
        nonlocal latest_result
        with result_lock:
            latest_result = result

    options = GestureRecognizerOptions(
        base_options=BaseOptions(model_asset_path=str(model_path)),
        running_mode=VisionRunningMode.LIVE_STREAM,
        num_hands=2,
        result_callback=on_result,
    )

    recognizer = GestureRecognizer.create_from_options(options)

    print("[Gesture] Running. Press ESC to exit.")
    print(f"[MQTT] Topic: {args.topic}")

    while True:
        ret, frame = cam.read()
        if not ret:
            print("[Gesture] Failed to read frame")
            break

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        timestamp_ms = int((time.monotonic() - start_time) * 1000)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

        recognizer.recognize_async(mp_image, timestamp_ms)

        with result_lock:
            result_snap = latest_result

        display = draw_result(frame, result_snap)

        # Publish mapped robot commands for supported gestures.
        threshold = max(float(args.min_score), MIN_SCORE_REQUIRED)
        commands = extract_commands(result_snap, threshold)
        now = time.monotonic()
        for payload in commands:
            command = payload["command"]
            last_ts = last_publish_ts_by_command.get(command, 0.0)
            publish_due = (now - last_ts) >= args.publish_period
            if publish_due:
                mqtt_pub.publish_json(payload)
                last_publish_ts_by_command[command] = now

        cv2.putText(
            display,
            f"MQTT: {'ON' if mqtt_pub.connected else 'OFF'}",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0) if mqtt_pub.connected else (0, 0, 255),
            2,
        )
        cv2.imshow("Gesture Tracker", display)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cam.release()
    cv2.destroyAllWindows()
    mqtt_pub.stop()


if __name__ == "__main__":
    main()
