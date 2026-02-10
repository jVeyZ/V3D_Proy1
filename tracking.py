"""
tracking.py - Seguimiento de objetos a lo largo de la secuencia de imágenes.

El seguimiento reutiliza la información de detecciones previas para localizar
el objeto de manera eficiente en frames sucesivos.

Métodos disponibles:
  - ColorTracker: seguimiento por color con ventana de búsqueda.
  - OpenCVTracker: wrapper sobre trackers de OpenCV (CSRT, KCF).
"""

import cv2
import numpy as np
import config


class ColorTracker:
    """
    Seguimiento basado en color HSV con ventana de búsqueda adaptativa.
    
    Utiliza la detección por color restringida a una región de interés (ROI)
    alrededor de la última posición conocida del objeto.
    """

    def __init__(self, hsv_lower=None, hsv_upper=None):
        self.hsv_lower = hsv_lower if hsv_lower is not None else config.BALL_HSV_LOWER
        self.hsv_upper = hsv_upper if hsv_upper is not None else config.BALL_HSV_UPPER
        self.last_position = None   # (cx, cy)
        self.last_radius = None     # radio en px
        self.search_margin = config.TRACKING_SEARCH_MARGIN
        self._initialized = False
        self._lost_counter = 0
        self._max_lost_frames = 30
        self._velocity = np.array([0.0, 0.0])  # velocidad estimada (px/frame)

    def initialize(self, cx, cy, radius):
        """Inicializa el tracker con la detección inicial."""
        self.last_position = np.array([float(cx), float(cy)])
        self.last_radius = float(radius)
        self._initialized = True
        self._lost_counter = 0
        self._velocity = np.array([0.0, 0.0])

    @property
    def is_initialized(self):
        return self._initialized

    @property
    def is_lost(self):
        return self._lost_counter > self._max_lost_frames

    def update(self, frame):
        """
        Actualiza la posición del objeto en el nuevo frame.
        
        Args:
            frame: imagen BGR
            
        Returns:
            (cx, cy, radius) o None si se perdió el objeto.
        """
        if not self._initialized:
            return None

        h, w = frame.shape[:2]

        # Predecir posición con velocidad
        predicted = self.last_position + self._velocity

        # Definir ROI alrededor de la posición predicha
        margin = self.search_margin + int(self.last_radius * 2)
        x1 = max(0, int(predicted[0] - margin))
        y1 = max(0, int(predicted[1] - margin))
        x2 = min(w, int(predicted[0] + margin))
        y2 = min(h, int(predicted[1] + margin))

        if x2 - x1 < 10 or y2 - y1 < 10:
            self._lost_counter += 1
            return None

        roi = frame[y1:y2, x1:x2]

        # Detección por color en la ROI
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self._lost_counter += 1
            if self._lost_counter <= self._max_lost_frames:
                # Mantener última posición conocida
                return (int(self.last_position[0]),
                        int(self.last_position[1]),
                        int(self.last_radius))
            return None

        # Buscar el contorno más parecido a la pelota
        best_candidate = None
        best_score = -1

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < np.pi * (config.BALL_MIN_RADIUS_PX ** 2) * 0.3:
                continue

            (cx_local, cy_local), radius = cv2.minEnclosingCircle(cnt)

            if radius < config.BALL_MIN_RADIUS_PX * 0.5:
                continue
            if radius > config.BALL_MAX_RADIUS_PX * 1.5:
                continue

            # Posición global
            cx_global = cx_local + x1
            cy_global = cy_local + y1

            # Puntuación: cercanía a posición predicha + similitud de radio
            dist = np.sqrt((cx_global - predicted[0]) ** 2 +
                           (cy_global - predicted[1]) ** 2)
            radius_diff = abs(radius - self.last_radius) / max(self.last_radius, 1)

            # Circularidad
            perimeter = cv2.arcLength(cnt, True)
            circularity = (4 * np.pi * area / (perimeter ** 2)
                           if perimeter > 0 else 0)

            score = circularity - dist / margin - radius_diff * 0.5
            if score > best_score:
                best_score = score
                best_candidate = (cx_global, cy_global, radius)

        if best_candidate is None:
            self._lost_counter += 1
            return (int(self.last_position[0]),
                    int(self.last_position[1]),
                    int(self.last_radius))

        cx, cy, radius = best_candidate
        new_position = np.array([cx, cy])

        # Actualizar velocidad (suavizada)
        alpha = 0.6
        new_vel = new_position - self.last_position
        self._velocity = alpha * new_vel + (1 - alpha) * self._velocity

        # Actualizar estado
        self.last_position = new_position
        self.last_radius = 0.7 * self.last_radius + 0.3 * radius  # suavizar radio
        self._lost_counter = 0

        return (int(cx), int(cy), int(self.last_radius))

    def get_roi(self):
        """Devuelve la ROI actual del tracker."""
        if self.last_position is None:
            return None
        margin = self.search_margin + int(self.last_radius * 2)
        cx, cy = self.last_position
        return (int(cx - margin), int(cy - margin),
                int(margin * 2), int(margin * 2))


class OpenCVTracker:
    """
    Wrapper sobre los trackers de OpenCV (CSRT, KCF, etc.).
    
    Utiliza la bounding box de la detección inicial para inicializar
    el tracker de OpenCV, que luego realiza el seguimiento frame a frame.
    """

    def __init__(self, method="csrt"):
        self.method = method.lower()
        self.tracker = None
        self._initialized = False
        self._lost_counter = 0
        self.last_position = None
        self.last_radius = None

    def _create_tracker(self):
        if self.method == "csrt":
            return cv2.TrackerCSRT_create()
        elif self.method == "kcf":
            return cv2.TrackerKCF_create()
        elif self.method == "mosse":
            return cv2.legacy.TrackerMOSSE_create()
        else:
            return cv2.TrackerCSRT_create()

    def initialize(self, cx, cy, radius, frame=None):
        """Inicializa el tracker con la detección y el frame."""
        if frame is None:
            self.last_position = np.array([float(cx), float(cy)])
            self.last_radius = float(radius)
            return

        self.tracker = self._create_tracker()
        bbox = (int(cx - radius), int(cy - radius),
                int(radius * 2), int(radius * 2))

        # Asegurar que bbox está dentro de la imagen
        h, w = frame.shape[:2]
        bbox = (max(0, bbox[0]), max(0, bbox[1]),
                min(bbox[2], w - bbox[0]), min(bbox[3], h - bbox[1]))

        self.tracker.init(frame, bbox)
        self.last_position = np.array([float(cx), float(cy)])
        self.last_radius = float(radius)
        self._initialized = True

    @property
    def is_initialized(self):
        return self._initialized

    @property
    def is_lost(self):
        return self._lost_counter > 30

    def update(self, frame):
        """Actualiza la posición usando el tracker de OpenCV."""
        if not self._initialized or self.tracker is None:
            return None

        success, bbox = self.tracker.update(frame)

        if success:
            x, y, w, h = [int(v) for v in bbox]
            cx = x + w // 2
            cy = y + h // 2
            radius = max(w, h) // 2

            self.last_position = np.array([float(cx), float(cy)])
            self.last_radius = float(radius)
            self._lost_counter = 0

            return (cx, cy, radius)
        else:
            self._lost_counter += 1
            if self.last_position is not None:
                return (int(self.last_position[0]),
                        int(self.last_position[1]),
                        int(self.last_radius))
            return None


def create_tracker(method=None):
    """Factory function para crear un tracker según la configuración."""
    if method is None:
        method = config.TRACKING_METHOD

    if method == "color":
        return ColorTracker()
    elif method in ("csrt", "kcf", "mosse"):
        return OpenCVTracker(method)
    else:
        return ColorTracker()