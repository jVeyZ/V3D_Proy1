"""
ar_viewer.py - Visor de Realidad Aumentada.

Apartado 1.3: Muestra las imágenes capturadas por la cámara con las 
              proyecciones de los elementos virtuales superpuestos.
"""

import cv2
import numpy as np
import config


class ARViewer:
    """
    Superpone elementos virtuales (hoyo, puntuación, trayectoria, etc.) 
    sobre las imágenes capturadas por la cámara.
    """

    def __init__(self, calibrator):
        """
        Args:
            calibrator: HomographyCalibrator calibrado (para proyectar 
                       puntos mundo → imagen)
        """
        self.calibrator = calibrator
        self.trail_image_points = []
        self._max_trail = 200

    def draw(self, frame, game_state=None, ball_detection=None):
        """
        Dibuja todos los elementos AR sobre el frame de la cámara.
        
        Args:
            frame: imagen BGR de la cámara
            game_state: diccionario con el estado del juego
            ball_detection: (cx, cy, radius) detección de la pelota en imagen
            
        Returns:
            frame con elementos AR superpuestos
        """
        output = frame.copy()

        if not self.calibrator.is_calibrated:
            cv2.putText(output, "No calibrado", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            return output

        # Dibujar área de juego
        self._draw_play_area(output)

        if game_state is not None:
            # Dibujar hoyo virtual
            self._draw_hole(output, game_state.get('hole_position'))

            # Dibujar obstáculos virtuales
            self._draw_obstacles(output, game_state.get('obstacles', []))

            # Dibujar trayectoria
            self._draw_trail(output)

            # Dibujar información del juego
            self._draw_game_info(output, game_state)

        # Dibujar detección de la pelota
        if ball_detection is not None:
            self._draw_ball_detection(output, ball_detection)

        return output

    def _draw_play_area(self, frame):
        """Dibuja el contorno del área de juego."""
        corners_world = config.WORLD_CORNERS
        corners_image = []

        for corner in corners_world:
            pt = self.calibrator.world_to_image(corner)
            if pt is not None:
                corners_image.append(pt.astype(int))

        if len(corners_image) == 4:
            pts = np.array(corners_image, dtype=np.int32)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

    def _draw_hole(self, frame, hole_position):
        """Dibuja el hoyo virtual sobre la imagen."""
        if hole_position is None:
            return

        center = self.calibrator.world_to_image(hole_position)
        if center is None:
            return

        center = tuple(center.astype(int))

        # Calcular radio del hoyo en píxeles (aproximado)
        offset_point = hole_position + np.array([config.HOLE_RADIUS_CM, 0])
        offset_px = self.calibrator.world_to_image(offset_point)
        if offset_px is not None:
            radius_px = int(np.linalg.norm(offset_px - np.array(center)))
        else:
            radius_px = 20

        # Dibujar hoyo (círculo oscuro con borde)
        overlay = frame.copy()
        cv2.circle(overlay, center, radius_px, config.AR_HOLE_COLOR_BGR, -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
        cv2.circle(frame, center, radius_px, config.AR_HOLE_BORDER_COLOR_BGR, 2)

        # Bandera
        flag_top = (center[0], center[1] - radius_px * 3)
        cv2.line(frame, center, flag_top, (200, 200, 200), 2)
        flag_pts = np.array([
            [flag_top[0], flag_top[1]],
            [flag_top[0] + 20, flag_top[1] + 8],
            [flag_top[0], flag_top[1] + 16]
        ], dtype=np.int32)
        cv2.fillPoly(frame, [flag_pts], config.AR_FLAG_COLOR_BGR)

    def _draw_obstacles(self, frame, obstacles):
        """Dibuja los obstáculos virtuales."""
        for (ox, oy, orad) in obstacles:
            center = self.calibrator.world_to_image(np.array([ox, oy]))
            if center is None:
                continue

            center = tuple(center.astype(int))

            offset = self.calibrator.world_to_image(np.array([ox + orad, oy]))
            if offset is not None:
                radius_px = int(np.linalg.norm(offset - np.array(center)))
            else:
                radius_px = 15

            overlay = frame.copy()
            cv2.circle(overlay, center, radius_px, config.AR_OBSTACLE_COLOR_BGR, -1)
            cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
            cv2.circle(frame, center, radius_px, (0, 0, 100), 2)

    def _draw_trail(self, frame):
        """Dibuja la trayectoria de la pelota."""
        if len(self.trail_image_points) < 2:
            return

        for i in range(1, len(self.trail_image_points)):
            alpha = i / len(self.trail_image_points)
            color = (0, int(255 * alpha), int(255 * alpha))
            thickness = max(1, int(2 * alpha))
            pt1 = tuple(self.trail_image_points[i - 1].astype(int))
            pt2 = tuple(self.trail_image_points[i].astype(int))
            cv2.line(frame, pt1, pt2, color, thickness)

    def _draw_ball_detection(self, frame, detection):
        """Dibuja la detección de la pelota."""
        cx, cy, radius = detection

        # Círculo de detección
        cv2.circle(frame, (cx, cy), int(radius), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)

        # Cruz en el centro
        cross_size = 10
        cv2.line(frame, (cx - cross_size, cy), (cx + cross_size, cy),
                 (0, 255, 0), 1)
        cv2.line(frame, (cx, cy - cross_size), (cx, cy + cross_size),
                 (0, 255, 0), 1)

    def _draw_game_info(self, frame, game_state):
        """Dibuja la información del juego (puntuación, etc.)."""
        h, w = frame.shape[:2]
        y_offset = 30

        # Panel semitransparente
        overlay = frame.copy()
        cv2.rectangle(overlay, (w - 250, 0), (w, 140), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

        # Información
        level = game_state.get('level', 1)
        putts = game_state.get('putts', 0)
        total_score = game_state.get('total_score', 0)
        status = game_state.get('status', '')
        ball_pos = game_state.get('ball_world_pos')

        cv2.putText(frame, f"Hoyo: {level}",
                    (w - 240, y_offset), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2)
        y_offset += 30
        cv2.putText(frame, f"Golpes: {putts}",
                    (w - 240, y_offset), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2)
        y_offset += 30
        cv2.putText(frame, f"Total: {total_score}",
                    (w - 240, y_offset), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2)
        y_offset += 30

        if ball_pos is not None:
            cv2.putText(frame, f"Pos: ({ball_pos[0]:.1f}, {ball_pos[1]:.1f})",
                        (w - 240, y_offset), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (200, 200, 200), 1)

        # Estado del juego
        if status:
            cv2.putText(frame, status, (20, h - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

    def add_trail_point(self, image_point):
        """Añade un punto a la trayectoria en imagen."""
        if image_point is not None:
            self.trail_image_points.append(np.array(image_point, dtype=np.float64))
            if len(self.trail_image_points) > self._max_trail:
                self.trail_image_points = self.trail_image_points[-self._max_trail:]

    def clear_trail(self):
        """Limpia la trayectoria."""
        self.trail_image_points = []