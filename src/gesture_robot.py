"""gesture_robot.py - Hilo de gesture tracking + UDP para control de robot."""

import copy
import json
import socket
import threading
import time
from pathlib import Path

import cv2
import numpy as np

try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
except Exception:
    MEDIAPIPE_AVAILABLE = False


COMMAND_MAP = {
    ("right", "Pointing_Up"): "forward_right",
    ("right", "Victory"): "backward_right",
    ("right", "Closed_Fist"): "stop_right",
    ("left", "Pointing_Up"): "forward_left",
    ("left", "Victory"): "backward_left",
    ("left", "Closed_Fist"): "stop_left",
}


class GestureRobotController:
    def __init__(self, camera_index=2, model_path="assets/gesture_recognizer.task",
                 udp_host="10.196.11.199", udp_port=4210,
                 min_score=0.40, publish_period=0.12):
        self._last_pair = (None, None)          # ← NUEVO
        self._last_publish_ts = 0.0  
        self.camera_index = camera_index
        self.model_path = str(model_path)
        self.udp_host = udp_host
        self.udp_port = udp_port
        self.min_score = max(float(min_score), 0.40)
        self.publish_period = float(publish_period)

        self.enabled = MEDIAPIPE_AVAILABLE
        self.connected = False
        self._running = False
        self._thread = None

        self._last_payload = {
            "command_left": None,
            "command_right": None,
            "timestamp_ms": 0,
        }
        self._lock = threading.Lock()
        self._udp_socket = None
        self._cap = None

        # Para mostrar el frame con overlay
        self._latest_frame = None
        self._frame_lock = threading.Lock()

    def start(self):
        if not self.enabled:
            print("[GestureRobot] MediaPipe no disponible.")
            return False
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._udp_socket:
            try:
                self._udp_socket.close()
            except Exception:
                pass
        if self._cap:
            try:
                self._cap.release()
            except Exception:
                pass

    def get_payload(self):
        with self._lock:
            return self._last_payload.copy()

    def get_frame(self):
        """Devuelve el último frame con overlay para mostrar en ventana."""
        with self._frame_lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    def _init_udp(self):
        try:
            self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.connected = True
            print(f"[GestureRobot] UDP → {self.udp_host}:{self.udp_port}")
        except OSError as exc:
            print(f"[GestureRobot] UDP error: {exc}")
            self.enabled = False

    def _extract_commands(self, result):
        if result is None or not getattr(result, "gestures", None):
            return {"command_left": None, "command_right": None,
                    "timestamp_ms": int(time.time() * 1000)}

        best_by_hand = {}
        for i, hand_gestures in enumerate(result.gestures):
            if not hand_gestures:
                continue
            g = hand_gestures[0]
            if g.score < self.min_score:
                continue
            hand = self._get_hand_label(result, i)
            if hand is None:
                continue
            command = COMMAND_MAP.get((hand, g.category_name))
            if command is None:
                continue
            prev = best_by_hand.get(hand)
            if prev is None or g.score > prev["score"]:
                best_by_hand[hand] = {"command": command, "score": float(g.score)}

        return {
            "command_left": (best_by_hand.get("left") or {}).get("command"),
            "command_right": (best_by_hand.get("right") or {}).get("command"),
            "timestamp_ms": int(time.time() * 1000),
        }

    def _get_hand_label(self, result, hand_index):
        if (result is None or not getattr(result, "handedness", None)
                or hand_index >= len(result.handedness)
                or not result.handedness[hand_index]):
            return None
        return str(result.handedness[hand_index][0].category_name).lower()

    def _publish(self, payload):
        any_cmd = payload["command_left"] is not None or payload["command_right"] is not None
        pair = (payload["command_left"], payload["command_right"])
        changed = pair != self._last_pair
        now = time.monotonic()
        period_due = (now - self._last_publish_ts) >= self.publish_period
        if not any_cmd:
            return
        if not (changed or period_due):
            return
        if self.connected and self._udp_socket:
            data = json.dumps(payload).encode("utf-8")
            try:
                self._udp_socket.sendto(data, (self.udp_host, self.udp_port))
            except OSError as exc:
                print(f"[GestureRobot] UDP send error: {exc}")
        self._last_publish_ts = now
        self._last_pair = pair
        with self._lock:
            self._last_payload = payload

    # Conexiones de la mano según el esquema de MediaPipe (21 landmarks)
    _HAND_CONNECTIONS = [
        (0, 1), (1, 2), (2, 3), (3, 4),          # pulgar
        (0, 5), (5, 6), (6, 7), (7, 8),            # índice
        (0, 9), (9, 10), (10, 11), (11, 12),       # corazón
        (0, 13), (13, 14), (14, 15), (15, 16),     # anular
        (0, 17), (17, 18), (18, 19), (19, 20),     # meñique
        (5, 9), (9, 13), (13, 17),                  # palma
    ]

    def _draw_overlay(self, frame_bgr, result):
        """Dibuja landmarks y etiquetas de gestos sobre el frame con OpenCV puro."""
        out = frame_bgr.copy()
        h, w, _ = out.shape

        num_hands = len(result.hand_landmarks) if (result and getattr(result, "hand_landmarks", None)) else 0

        if num_hands == 0:
            cv2.putText(out, "HANDS: 0", (15, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        else:
            for i, landmarks in enumerate(result.hand_landmarks):
                # Convertir coordenadas normalizadas a píxeles
                pts = [(int(lm.x * w), int(lm.y * h)) for lm in landmarks]

                # Conexiones (huesos)
                for a, b in self._HAND_CONNECTIONS:
                    if a < len(pts) and b < len(pts):
                        cv2.line(out, pts[a], pts[b], (0, 210, 110), 2, cv2.LINE_AA)

                # Landmarks (articulaciones)
                for idx, pt in enumerate(pts):
                    # Fingertips más grandes
                    r = 6 if idx in (4, 8, 12, 16, 20) else 4
                    cv2.circle(out, pt, r, (0, 150, 255), -1, cv2.LINE_AA)
                    cv2.circle(out, pt, r, (255, 255, 255), 1, cv2.LINE_AA)

                # Etiqueta gesto + confianza encima de la mano
                if result.gestures and i < len(result.gestures) and result.gestures[i]:
                    gesture = result.gestures[i][0]
                    hand_label = self._get_hand_label(result, i) or "?"
                    label = f"[{hand_label}] {gesture.category_name} {gesture.score:.0%}"
                    tx = max(0, min(p[0] for p in pts))
                    ty = max(22, min(p[1] for p in pts) - 12)
                    # Fondo negro semitransparente para legibilidad
                    (tw, tgh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_DUPLEX, 0.65, 2)
                    cv2.rectangle(out, (tx - 2, ty - tgh - 4), (tx + tw + 2, ty + 4),
                                  (0, 0, 0), -1)
                    cv2.putText(out, label, (tx, ty),
                                cv2.FONT_HERSHEY_DUPLEX, 0.65, (88, 230, 54), 2, cv2.LINE_AA)

            cv2.putText(out, f"HANDS: {num_hands}", (15, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

        # Estado UDP
        cv2.putText(out, f"UDP: {'ON' if self.connected else 'OFF'}", (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 0) if self.connected else (0, 0, 255), 2)

        return out

    def _run(self):
        self._init_udp()
        if not self.enabled:
            return

        self._cap = cv2.VideoCapture(self.camera_index)
        if not self._cap.isOpened():
            print(f"[GestureRobot] No se pudo abrir cámara {self.camera_index}")
            self.enabled = False
            return

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        model_path = Path(self.model_path)
        if not model_path.exists():
            print(f"[GestureRobot] Modelo no encontrado: {model_path} – mostrando cámara sin gestos")
            # Capturar en bruto para que get_frame() devuelva frames igualmente
            while self._running:
                ret, frame = self._cap.read()
                if not ret:
                    time.sleep(0.001)
                    continue
                cv2.putText(frame, "MODEL NOT FOUND", (10, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                with self._frame_lock:
                    self._latest_frame = frame.copy()
                time.sleep(0.033)
            self._cap.release()
            return

        latest_result = None
        result_lock = threading.Lock()
        start_time = time.monotonic()

        def on_result(result, _output_image, _timestamp_ms):
            nonlocal latest_result
            with result_lock:
                latest_result = copy.deepcopy(result)

        options = mp.tasks.vision.GestureRecognizerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(model_path)),
            running_mode=mp.tasks.vision.RunningMode.LIVE_STREAM,
            num_hands=2,
            result_callback=on_result,
        )
        recognizer = mp.tasks.vision.GestureRecognizer.create_from_options(options)
        print("[GestureRobot] Gesture tracker activo.")

        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                time.sleep(0.001)
                continue

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            ts_ms = int((time.monotonic() - start_time) * 1000)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb.copy())

            try:
                recognizer.recognize_async(mp_image, ts_ms)
            except RuntimeError:
                continue

            with result_lock:
                result_snap = latest_result

            payload = self._extract_commands(result_snap)
            self._publish(payload)

            # Dibujar overlay y guardar frame para la ventana principal
            display_frame = self._draw_overlay(frame, result_snap)
            with self._frame_lock:
                self._latest_frame = display_frame

        recognizer.close()