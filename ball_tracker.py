"""
green_ball_tracker.py - Tracking de bola verde con deque + contornos.
Basado en: https://pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
Adaptado para stereo con Kalman smoothing.
"""

from collections import deque
import numpy as np
import cv2
import game_config as  config


class GreenBallTracker:
    """
    Tracker de bola verde usando el método de PyImageSearch:
    - Gaussian blur + HSV threshold
    - Erode/dilate para limpiar máscara
    - Contorno más grande → círculo mínimo + centroide
    - Deque para trail visual
    - Kalman para suavizado de posición 3D
    """

    def __init__(self, trail_size=None):
        self.trail_size = trail_size or config.TRAIL_BUFFER_SIZE
        self.pts = deque(maxlen=self.trail_size)
        self._kalman = None
        self._initialized = False
        self._lost_counter = 0
        self._max_lost = 60
        self._confirm_needed = 2
        self._confirm_counter = 0
        self._pending = None
        self._last_valid = None
        self._last_velocity = np.array([0.0, 0.0], dtype=np.float32)

    def _init_kalman(self, cx, cy):
        """Inicializa filtro Kalman para suavizar posición."""
        self._kalman = cv2.KalmanFilter(4, 2)
        self._kalman.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)
        self._kalman.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)
        self._kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self._kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.1
        self._kalman.errorCovPost = np.eye(4, dtype=np.float32) * 100
        self._kalman.statePost = np.array([[cx], [cy], [0], [0]], dtype=np.float32)
        self._initialized = True
        self._lost_counter = 0

    @property
    def is_initialized(self):
        return self._initialized

    @property
    def is_lost(self):
        return self._lost_counter > self._max_lost

    def detect(self, frame):
        """
        Detección de bola verde usando el pipeline de PyImageSearch.
        
        Returns:
            (cx, cy, radius) o None
        """
        # Gaussian blur + HSV
        blurred = cv2.GaussianBlur(frame, config.GAUSSIAN_BLUR_SIZE, 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Máscara verde
        mask = cv2.inRange(hsv, config.BALL_GREEN_HSV_LOWER, config.BALL_GREEN_HSV_UPPER)
        mask = cv2.erode(mask, None, iterations=config.ERODE_ITERATIONS)
        mask = cv2.dilate(mask, None, iterations=config.DILATE_ITERATIONS)

        # Contornos
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        if len(cnts) == 0:
            return None

        # Contorno más grande
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        if radius < config.MIN_RADIUS:
            return None

        # Centroide
        M = cv2.moments(c)
        if M["m00"] == 0:
            return None

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        return (cx, cy, int(radius))

    def update(self, frame):
        """
        Actualiza tracking: detecta + suaviza con Kalman + actualiza trail.
        
        Returns:
            (cx, cy, radius) suavizado o None si perdido
        """
        detection = self.detect(frame)

        if detection is None:
            self._lost_counter += 1
            if self._kalman is not None and self._lost_counter <= self._max_lost:
                # Predicción Kalman cuando se pierde
                pred = self._kalman.predict()
                cx = int(pred[0, 0])
                cy = int(pred[1, 0])
                # Radio estimado del último válido
                last_r = self.pts[-1][2] if self.pts else config.MIN_RADIUS
                self.pts.appendleft((cx, cy, last_r))
                self._last_valid = np.array([cx, cy], dtype=np.float32)
                return (cx, cy, last_r)
            if self._last_valid is not None and self._lost_counter <= self._max_lost:
                pred = self._last_valid + self._last_velocity
                cx = int(pred[0])
                cy = int(pred[1])
                last_r = self.pts[-1][2] if self.pts else config.MIN_RADIUS
                self.pts.appendleft((cx, cy, last_r))
                self._last_valid = pred
                return (cx, cy, last_r)
            return None

        cx, cy, radius = detection

        if not self._initialized:
            if self._pending is None:
                self._pending = (cx, cy, radius)
                self._confirm_counter = 1
                return (cx, cy, radius)
            px, py, pr = self._pending
            if abs(cx - px) <= 25 and abs(cy - py) <= 25:
                self._confirm_counter += 1
            else:
                self._pending = (cx, cy, radius)
                self._confirm_counter = 1
            if self._confirm_counter >= self._confirm_needed:
                self._init_kalman(cx, cy)
                self.pts.appendleft((cx, cy, radius))
                self._last_valid = np.array([cx, cy], dtype=np.float32)
                self._last_velocity = np.array([0.0, 0.0], dtype=np.float32)
                return (cx, cy, radius)
            return (cx, cy, radius)

        # Predicción y verificación
        pred = self._kalman.predict()
        pred_cx = float(pred[0, 0])
        pred_cy = float(pred[1, 0])
        if abs(cx - pred_cx) > 80 or abs(cy - pred_cy) > 80:
            self._lost_counter += 1
            last_r = self.pts[-1][2] if self.pts else config.MIN_RADIUS
            self.pts.appendleft((int(pred_cx), int(pred_cy), last_r))
            self._last_valid = np.array([pred_cx, pred_cy], dtype=np.float32)
            return (int(pred_cx), int(pred_cy), last_r)

        # Corrección Kalman
        measurement = np.array([[cx], [cy]], dtype=np.float32)
        self._kalman.correct(measurement)
        pred = self._kalman.predict()

        # Usar posición suavizada por Kalman
        smooth_cx = int(pred[0, 0])
        smooth_cy = int(pred[1, 0])

        self._lost_counter = 0
        if self._last_valid is not None:
            vel = np.array([smooth_cx, smooth_cy], dtype=np.float32) - self._last_valid
            self._last_velocity = vel
        self._last_valid = np.array([smooth_cx, smooth_cy], dtype=np.float32)
        self.pts.appendleft((smooth_cx, smooth_cy, radius))

        return (smooth_cx, smooth_cy, radius)

    def draw_trail(self, frame):
        """Dibuja el trail de la bola sobre el frame."""
        for i in range(1, len(self.pts)):
            if self.pts[i - 1] is None or self.pts[i] is None:
                continue
            thickness = int(np.sqrt(self.trail_size / float(i + 1)) * 2.5)
            cv2.line(frame, (self.pts[i][0], self.pts[i][1]),
                     (self.pts[i - 1][0], self.pts[i - 1][1]),
                     (0, 0, 255), thickness)

    def draw_detection(self, frame, detection):
        """Dibuja círculo y centroide de la detección."""
        if detection is None:
            return
        cx, cy, radius = detection
        cv2.circle(frame, (cx, cy), int(radius), (0, 255, 255), 2)
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

    def reset(self):
        """Reinicia el tracker."""
        self.pts.clear()
        self._initialized = False
        self._kalman = None
        self._lost_counter = 0
        self._confirm_counter = 0
        self._pending = None
        self._last_valid = None
        self._last_velocity = np.array([0.0, 0.0], dtype=np.float32)


class StereoGreenTracker:
    """
    Par de trackers para stereo: uno por cámara.
    """

    def __init__(self):
        self.left = GreenBallTracker()
        self.right = GreenBallTracker()

    def detect_both(self, frame_left, frame_right):
        """Detecta en ambas cámaras (inicialización)."""
        det_left = self.left.detect(frame_left)
        det_right = self.right.detect(frame_right)

        if det_left is not None and det_right is not None:
            self.left._init_kalman(*det_left[:2])
            self.left.pts.appendleft(det_left)
            self.right._init_kalman(*det_right[:2])
            self.right.pts.appendleft(det_right)
            return det_left, det_right
        return None, None

    def update(self, frame_left, frame_right):
        """Actualiza ambos trackers."""
        return self.left.update(frame_left), self.right.update(frame_right)

    def reset(self):
        self.left.reset()
        self.right.reset()


    @property
    def is_initialized(self):
        return self.left.is_initialized and self.right.is_initialized