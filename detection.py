"""
detection.py - Detección de objetos (pelota) en la imagen.

Dos modos:
  - Manual: el usuario hace clic en el centro de la pelota.
  - Automático: detección por color (HSV) + contornos / Hough circles.
"""

import cv2
import numpy as np
import config


class ManualDetector:
    """Detecta la pelota mediante clic del usuario en la imagen."""

    def __init__(self):
        self._clicked_point = None
        self._detection_done = False

    def detect(self, frame):
        """
        Muestra la imagen y espera un clic del usuario.
        
        Returns:
            (x, y, radius) en píxeles, o None si se cancela.
        """
        self._clicked_point = None
        self._detection_done = False
        window_name = "Deteccion Manual: clic en el centro de la pelota"

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(window_name, self._mouse_callback)

        while not self._detection_done:
            display = frame.copy()
            cv2.putText(display,
                        "Haz clic en el centro de la pelota. ESC para cancelar.",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            if self._clicked_point is not None:
                cv2.circle(display, self._clicked_point, 5, (0, 0, 255), -1)
                cv2.circle(display, self._clicked_point, 30, (0, 255, 0), 2)

            cv2.imshow(window_name, display)
            key = cv2.waitKey(30) & 0xFF
            if key == 27:
                cv2.destroyWindow(window_name)
                return None
            elif key == 13 or key == 32:  # Enter o Espacio para confirmar
                if self._clicked_point is not None:
                    self._detection_done = True

        cv2.destroyWindow(window_name)
        x, y = self._clicked_point

        # Estimar el radio por color alrededor del punto clicado
        radius = self._estimate_radius(frame, x, y)
        return (x, y, radius)

    def _mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self._clicked_point = (x, y)
        elif event == cv2.EVENT_LBUTTONDBLCLK:
            self._clicked_point = (x, y)
            self._detection_done = True

    def _estimate_radius(self, frame, cx, cy, default=25):
        """Estima el radio de la pelota por análisis de color alrededor del clic."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Tomar color del punto central
        h_center = hsv[max(0, cy), max(0, cx)]

        # Crear máscara con rango centrado en el color del clic
        margin_h, margin_s, margin_v = 15, 60, 60
        lower = np.array([
            max(0, int(h_center[0]) - margin_h),
            max(0, int(h_center[1]) - margin_s),
            max(0, int(h_center[2]) - margin_v)
        ])
        upper = np.array([
            min(179, int(h_center[0]) + margin_h),
            min(255, int(h_center[1]) + margin_s),
            min(255, int(h_center[2]) + margin_v)
        ])

        mask = cv2.inRange(hsv, lower, upper)

        # Buscar contorno más cercano al clic
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        best_radius = default
        best_dist = float('inf')

        for cnt in contours:
            (x, y), r = cv2.minEnclosingCircle(cnt)
            if config.BALL_MIN_RADIUS_PX < r < config.BALL_MAX_RADIUS_PX:
                dist = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
                if dist < best_dist:
                    best_dist = dist
                    best_radius = r

        return int(best_radius)

    # --------------------------------------------------------
    # Non-blocking manual detection helper
    # --------------------------------------------------------
    def detect_point(self, frame, x, y):
        """Detecta la pelota asumiendo que el usuario ha pulsado en (x,y).

        Esta función NO abre ventanas ni bloquea; devuelve (x, y, radius)
        estimado o None si no se encuentra un radio razonable.
        """
        h, w = frame.shape[:2]
        if x < 0 or y < 0 or x >= w or y >= h:
            return None

        radius = self._estimate_radius(frame, x, y)
        if radius is None or radius < config.BALL_MIN_RADIUS_PX:
            return None
        return (int(x), int(y), int(radius))

class AutomaticDetector:
    """
    Detección automática de la pelota por color (HSV) y forma circular.
    
    Combina umbralización HSV, operaciones morfológicas, detección de contornos
    y filtrado por circularidad.
    """

    def __init__(self, hsv_lower=None, hsv_upper=None):
        self.hsv_lower = hsv_lower if hsv_lower is not None else config.BALL_HSV_LOWER
        self.hsv_upper = hsv_upper if hsv_upper is not None else config.BALL_HSV_UPPER
        self._debug = False

    def detect(self, frame, roi=None):
        """
        Detecta la pelota en la imagen completa o dentro de una ROI.
        
        Args:
            frame: imagen BGR
            roi: (x, y, w, h) región de interés opcional
            
        Returns:
            (cx, cy, radius) del mejor candidato, o None si no se detecta.
        """
        if roi is not None:
            rx, ry, rw, rh = roi
            sub_frame = frame[ry:ry + rh, rx:rx + rw]
            result = self._detect_in_image(sub_frame)
            if result is not None:
                cx, cy, r = result
                return (cx + rx, cy + ry, r)
            return None
        else:
            return self._detect_in_image(frame)

    def _detect_in_image(self, image):
        """Detección interna sobre una imagen."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Máscara por color
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # Si el rango cruza el 0 en H (para rojos), combinar dos rangos
        if self.hsv_lower[0] > self.hsv_upper[0]:
            mask1 = cv2.inRange(hsv,
                                np.array([self.hsv_lower[0], self.hsv_lower[1],
                                          self.hsv_lower[2]]),
                                np.array([179, self.hsv_upper[1],
                                          self.hsv_upper[2]]))
            mask2 = cv2.inRange(hsv,
                                np.array([0, self.hsv_lower[1],
                                          self.hsv_lower[2]]),
                                np.array([self.hsv_upper[0], self.hsv_upper[1],
                                          self.hsv_upper[2]]))
            mask = mask1 | mask2

        # Operaciones morfológicas para limpiar
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        if self._debug:
            cv2.imshow("Mascara deteccion", mask)

        # Buscar contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return self._fallback_hough(image, mask)

        # Filtrar y puntuar candidatos
        candidates = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < np.pi * config.BALL_MIN_RADIUS_PX ** 2:
                continue

            (cx, cy), radius = cv2.minEnclosingCircle(cnt)

            if not (config.BALL_MIN_RADIUS_PX < radius < config.BALL_MAX_RADIUS_PX):
                continue

            # Circularidad
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter * perimeter)

            # Ratio de llenado del círculo
            circle_area = np.pi * radius * radius
            fill_ratio = area / circle_area if circle_area > 0 else 0

            # Puntuación combinada
            score = circularity * 0.5 + fill_ratio * 0.3 + (area / 10000) * 0.2

            candidates.append((cx, cy, radius, score))

        if not candidates:
            return self._fallback_hough(image, mask)

        # Seleccionar el mejor candidato
        best = max(candidates, key=lambda c: c[3])
        return (int(best[0]), int(best[1]), int(best[2]))

    def _fallback_hough(self, image, mask):
        """Detección alternativa con HoughCircles."""
        circles = cv2.HoughCircles(
            mask, cv2.HOUGH_GRADIENT, dp=1.2,
            minDist=50,
            param1=100, param2=30,
            minRadius=config.BALL_MIN_RADIUS_PX,
            maxRadius=config.BALL_MAX_RADIUS_PX
        )

        if circles is not None:
            circles = np.round(circles[0, :]).astype(int)
            # Tomar el más grande
            best = max(circles, key=lambda c: c[2])
            return (best[0], best[1], best[2])

        return None

    def detect_multiple(self, frame, max_objects=5):
        """Detecta múltiples objetos (para extensiones futuras)."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        results = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < np.pi * config.BALL_MIN_RADIUS_PX ** 2:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            if config.BALL_MIN_RADIUS_PX < radius < config.BALL_MAX_RADIUS_PX:
                results.append((int(cx), int(cy), int(radius)))

        results.sort(key=lambda c: c[2], reverse=True)
        return results[:max_objects]

    def set_debug(self, debug=True):
        self._debug = debug