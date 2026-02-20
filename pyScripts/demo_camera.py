"""
demo_camera.py - Cámara simulada para pruebas sin hardware.

Genera frames sintéticos que simulan una vista cenital (o perspectiva) 
de una mesa de mini-golf con una pelota que se puede mover con el ratón
o el teclado. Permite probar todo el pipeline (calibración, detección,
seguimiento, posicionamiento, juego) sin necesidad de una cámara real.

Modos de uso:
  - Modo ratón: mover la pelota arrastrando con el ratón.
  - Modo teclado: mover la pelota con WASD / flechas.
  - Modo automático: la pelota sigue una trayectoria predefinida.
"""

import cv2
import numpy as np
import config


class DemoCamera:
    """
    Cámara simulada que genera frames con una pelota sintética sobre
    un fondo que simula una mesa de mini-golf.
    
    La cámara simula una vista con perspectiva leve, donde las 4 esquinas
    del área de juego se conocen tanto en coordenadas mundo como en
    coordenadas de imagen, facilitando la calibración automática.
    """

    def __init__(self, width=None, height=None):
        self.width = width or config.CAMERA_WIDTH
        self.height = height or config.CAMERA_HEIGHT

        # Posición de la pelota en coordenadas del mundo (cm)
        self.ball_world_pos = np.array([30.0, 20.0], dtype=np.float64)
        self.ball_velocity = np.array([0.0, 0.0], dtype=np.float64)

        # Simular una perspectiva: definir las 4 esquinas del área de juego
        # en coordenadas de imagen con algo de perspectiva
        margin_x = int(self.width * 0.1)
        margin_y = int(self.height * 0.1)

        # Perspectiva leve: las esquinas superiores están más juntas
        perspective_shrink = 0.08
        top_shrink_x = int(self.width * perspective_shrink)
        top_shrink_y = int(self.height * perspective_shrink * 0.5)

        self.image_corners = np.array([
            [margin_x + top_shrink_x, margin_y + top_shrink_y],           # TL
            [self.width - margin_x - top_shrink_x, margin_y + top_shrink_y],  # TR
            [self.width - margin_x, self.height - margin_y],               # BR
            [margin_x, self.height - margin_y],                            # BL
        ], dtype=np.float64)

        # Homografía mundo → imagen (para renderizar la pelota)
        self.H_world_to_image, _ = cv2.findHomography(
            config.WORLD_CORNERS, self.image_corners
        )
        # Homografía imagen → mundo (para verificar)
        self.H_image_to_world, _ = cv2.findHomography(
            self.image_corners, config.WORLD_CORNERS
        )

        # Fondo pre-renderizado
        self._background = None
        self._build_background()

        # Estado
        self._is_open = True
        self._mouse_dragging = False
        self._frame_count = 0

        # Trayectoria automática
        self._auto_mode = False
        self._auto_waypoints = [
            np.array([10.0, 10.0]),
            np.array([50.0, 10.0]),
            np.array([50.0, 30.0]),
            np.array([30.0, 20.0]),
            np.array([15.0, 35.0]),
            np.array([45.0, 20.0]),  # Hacia el hoyo nivel 1
        ]
        self._auto_wp_index = 0
        self._auto_speed = 0.5  # cm/frame

    def _build_background(self):
        """Construye la imagen de fondo (mesa de juego vista desde arriba)."""
        bg = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Fondo general oscuro
        bg[:] = (60, 60, 60)

        # Dibujar la mesa verde con perspectiva
        pts = self.image_corners.astype(np.int32)
        cv2.fillPoly(bg, [pts], config.DEMO_BG_COLOR_BGR)

        # Borde de la mesa
        cv2.polylines(bg, [pts], True, (30, 80, 200), 3)

        # Patrón de cesped (líneas sutiles)
        for i in range(0, 20):
            t = i / 20.0
            # Líneas horizontales en el mundo → perspectiva en imagen
            p1 = self._world_to_image_pt(
                np.array([0, t * config.PLAY_AREA_HEIGHT])
            )
            p2 = self._world_to_image_pt(
                np.array([config.PLAY_AREA_WIDTH, t * config.PLAY_AREA_HEIGHT])
            )
            if p1 is not None and p2 is not None:
                cv2.line(bg, tuple(p1.astype(int)), tuple(p2.astype(int)),
                         (45, 145, 45), 1)

        # Marcas de referencia en las esquinas (simulando marcadores ArUco)
        for i, corner in enumerate(self.image_corners):
            cx, cy = int(corner[0]), int(corner[1])
            size = 20
            # Cuadrado blanco con ID
            cv2.rectangle(bg, (cx - size, cy - size), (cx + size, cy + size),
                          (255, 255, 255), 2)
            # Cuadrado interior negro con patrón
            cv2.rectangle(bg, (cx - size + 4, cy - size + 4),
                          (cx + size - 4, cy + size - 4), (0, 0, 0), -1)
            # Número del marcador
            cv2.putText(bg, str(i), (cx - 6, cy + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        self._background = bg

    def _world_to_image_pt(self, world_pt):
        """Convierte punto mundo (cm) a punto imagen (px)."""
        if self.H_world_to_image is None:
            return None
        p = np.array([world_pt[0], world_pt[1], 1.0])
        q = self.H_world_to_image @ p
        if abs(q[2]) < 1e-10:
            return None
        return np.array([q[0] / q[2], q[1] / q[2]])

    def _image_to_world_pt(self, image_pt):
        """Convierte punto imagen (px) a punto mundo (cm)."""
        if self.H_image_to_world is None:
            return None
        p = np.array([image_pt[0], image_pt[1], 1.0])
        q = self.H_image_to_world @ p
        if abs(q[2]) < 1e-10:
            return None
        return np.array([q[0] / q[2], q[1] / q[2]])

    def read(self):
        """
        Lee un frame de la cámara simulada.
        
        Returns:
            (ret, frame): tupla como cv2.VideoCapture.read()
        """
        if not self._is_open:
            return False, None

        # Actualizar posición de la pelota si modo automático
        if self._auto_mode:
            self._update_auto()

        # Aplicar velocidad (con fricción)
        self.ball_world_pos += self.ball_velocity
        self.ball_velocity *= 0.95  # Fricción
        if np.linalg.norm(self.ball_velocity) < 0.01:
            self.ball_velocity = np.array([0.0, 0.0])

        # Limitar a los bordes
        r = config.BALL_REAL_RADIUS_CM
        self.ball_world_pos[0] = np.clip(
            self.ball_world_pos[0], r, config.PLAY_AREA_WIDTH - r
        )
        self.ball_world_pos[1] = np.clip(
            self.ball_world_pos[1], r, config.PLAY_AREA_HEIGHT - r
        )

        # Rebote en bordes
        if (self.ball_world_pos[0] <= r or
                self.ball_world_pos[0] >= config.PLAY_AREA_WIDTH - r):
            self.ball_velocity[0] *= -0.7
        if (self.ball_world_pos[1] <= r or
                self.ball_world_pos[1] >= config.PLAY_AREA_HEIGHT - r):
            self.ball_velocity[1] *= -0.7

        # Generar frame
        frame = self._render_frame()
        self._frame_count += 1

        return True, frame

    def _render_frame(self):
        """Genera el frame actual con la pelota en su posición."""
        frame = self._background.copy()

        # Dibujar la pelota
        ball_img = self._world_to_image_pt(self.ball_world_pos)
        if ball_img is not None:
            cx, cy = int(ball_img[0]), int(ball_img[1])

            # Calcular el radio en píxeles (depende de la perspectiva)
            offset_pt = self._world_to_image_pt(
                self.ball_world_pos + np.array([config.BALL_REAL_RADIUS_CM, 0])
            )
            if offset_pt is not None:
                radius_px = max(
                    5, int(np.linalg.norm(offset_pt - ball_img))
                )
            else:
                radius_px = 15

            # Pelota con sombra
            shadow_offset = 3
            cv2.circle(frame, (cx + shadow_offset, cy + shadow_offset),
                       radius_px, (20, 80, 20), -1)

            # Pelota principal (naranja)
            cv2.circle(frame, (cx, cy), radius_px,
                       config.DEMO_BALL_COLOR_BGR, -1)

            # Brillo
            highlight_offset = radius_px // 3
            cv2.circle(frame, (cx - highlight_offset, cy - highlight_offset),
                       max(2, radius_px // 4), (100, 200, 255), -1)

        # Info en pantalla
        cv2.putText(frame, "CAMARA SIMULADA", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1)
        cv2.putText(frame,
                    f"Pelota: ({self.ball_world_pos[0]:.1f}, "
                    f"{self.ball_world_pos[1]:.1f}) cm",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (200, 200, 200), 1)

        mode_text = "Auto" if self._auto_mode else "Manual (WASD/raton)"
        cv2.putText(frame, f"Modo: {mode_text}",
                    (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (200, 200, 200), 1)

        return frame

    def _update_auto(self):
        """Mueve la pelota automáticamente por la trayectoria."""
        if self._auto_wp_index >= len(self._auto_waypoints):
            self._auto_wp_index = 0

        target = self._auto_waypoints[self._auto_wp_index]
        direction = target - self.ball_world_pos
        dist = np.linalg.norm(direction)

        if dist < 1.0:
            self._auto_wp_index += 1
        else:
            direction = direction / dist
            self.ball_world_pos += direction * self._auto_speed

    def move_ball_keyboard(self, key):
        """
        Mueve la pelota con el teclado.
        
        Args:
            key: código de tecla (resultado de cv2.waitKey)
        """
        speed = config.DEMO_BALL_SPEED * 0.5  # cm/pulsación

        if key == ord('w') or key == 82:     # W o flecha arriba
            self.ball_velocity[1] -= speed
        elif key == ord('s') or key == 84:   # S o flecha abajo
            self.ball_velocity[1] += speed
        elif key == ord('a') or key == 81:   # A o flecha izquierda
            self.ball_velocity[0] -= speed
        elif key == ord('d') or key == 83:   # D o flecha derecha
            self.ball_velocity[0] += speed
        elif key == ord('t'):                 # T: toggle auto
            self._auto_mode = not self._auto_mode
            print(f"[Demo] Modo {'automático' if self._auto_mode else 'manual'}")

    def set_ball_position(self, world_pos):
        """Establece la posición de la pelota directamente."""
        self.ball_world_pos = np.array(world_pos, dtype=np.float64)
        self.ball_velocity = np.array([0.0, 0.0])

    def mouse_callback(self, event, x, y, flags, param):
        """Callback del ratón para mover la pelota arrastrando."""
        if event == cv2.EVENT_LBUTTONDOWN:
            self._mouse_dragging = True
            world_pt = self._image_to_world_pt(np.array([x, y]))
            if world_pt is not None:
                self.ball_world_pos = world_pt
                self.ball_velocity = np.array([0.0, 0.0])

        elif event == cv2.EVENT_MOUSEMOVE and self._mouse_dragging:
            world_pt = self._image_to_world_pt(np.array([x, y]))
            if world_pt is not None:
                old_pos = self.ball_world_pos.copy()
                self.ball_world_pos = world_pt
                self.ball_velocity = (self.ball_world_pos - old_pos) * 0.3

        elif event == cv2.EVENT_LBUTTONUP:
            self._mouse_dragging = False

    def get_calibration_corners(self):
        """
        Devuelve las esquinas de la imagen que corresponden al área de juego.
        Útil para calibración automática sin interacción.
        
        Returns:
            np.array de 4 puntos (TL, TR, BR, BL) en píxeles
        """
        return self.image_corners.copy()

    def isOpened(self):
        """Compatibilidad con cv2.VideoCapture."""
        return self._is_open

    def release(self):
        """Compatibilidad con cv2.VideoCapture."""
        self._is_open = False

    def get(self, prop):
        """Compatibilidad con cv2.VideoCapture.get()."""
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            return self.width
        elif prop == cv2.CAP_PROP_FRAME_HEIGHT:
            return self.height
        elif prop == cv2.CAP_PROP_FPS:
            return config.FPS
        return 0

    def set(self, prop, value):
        """Compatibilidad con cv2.VideoCapture.set()."""
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            self.width = int(value)
        elif prop == cv2.CAP_PROP_FRAME_HEIGHT:
            self.height = int(value)
