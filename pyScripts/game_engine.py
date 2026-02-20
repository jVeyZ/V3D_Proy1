"""
game_engine.py - Motor del juego de mini-golf.

Apartado 1.5: Aplicación de posicionamiento planar que integra la
              funcionalidad de detección, seguimiento, posicionamiento
              y corrección de altura en un juego interactivo de mini-golf
              (tipo "putter" sobre un green plano).

Funcionalidades:
  - Gestión de estado del juego (niveles, golpes, puntuación)
  - Detección de pelota embocada (hoyo)
  - Detección de pelota parada (conteo de golpes)
  - Colisiones con obstáculos
  - Progresión de niveles
"""

import numpy as np
import time
import config


class GameState:
    """Estado completo del juego en un instante."""

    def __init__(self):
        self.level = 1
        self.putts = 0
        self.total_score = 0
        self.status = ""
        self.ball_world_pos = None
        self.ball_image_pos = None
        self.hole_position = None
        self.obstacles = list(config.OBSTACLES)
        self.is_ball_moving = False
        self.is_hole_in = False
        self.game_over = False
        self.distance_to_hole = float('inf')

    def to_dict(self):
        """Convierte el estado a diccionario para el visor AR."""
        return {
            'level': self.level,
            'putts': self.putts,
            'total_score': self.total_score,
            'status': self.status,
            'ball_world_pos': self.ball_world_pos,
            'ball_image_pos': self.ball_image_pos,
            'hole_position': self.hole_position,
            'obstacles': self.obstacles,
            'is_ball_moving': self.is_ball_moving,
            'is_hole_in': self.is_hole_in,
            'game_over': self.game_over,
            'distance_to_hole': self.distance_to_hole,
        }


class MiniGolfEngine:
    """
    Motor de juego de mini-golf virtual.
    
    Mecánica:
      - La pelota real se mueve sobre la mesa (plano de trabajo).
      - Un hoyo virtual se proyecta en la mesa mediante AR.
      - El jugador golpea la pelota real con un putter.
      - Cuando la pelota se detiene sobre el hoyo, se cuenta como embocada.
      - Se progresa al siguiente nivel (nuevo hoyo).
    """

    def __init__(self):
        self.state = GameState()
        self._stopped_frame_count = 0
        self._last_positions = []
        self._max_history = config.BALL_STOPPED_FRAMES * 2
        self._level_start_time = 0
        self._hole_in_time = 0
        self._celebrating = False
        self._celebration_duration = 3.0  # segundos

    def start_game(self):
        """Inicia una nueva partida."""
        self.state = GameState()
        self.state.level = 1
        self.state.total_score = 0
        self._set_hole_for_level(1)
        self.state.status = "Nivel 1 - ¡A jugar!"
        self._level_start_time = time.time()
        self._celebrating = False
        print("[MiniGolf] ¡Partida iniciada! Nivel 1")

    def _set_hole_for_level(self, level):
        """Establece la posición del hoyo según el nivel."""
        idx = (level - 1) % len(config.HOLE_POSITIONS)
        self.state.hole_position = config.HOLE_POSITIONS[idx].copy()
        print(f"[MiniGolf] Hoyo del nivel {level}: "
              f"({self.state.hole_position[0]:.1f}, "
              f"{self.state.hole_position[1]:.1f}) cm")

    def update(self, ball_world_pos):
        """
        Actualiza el estado del juego con la nueva posición de la pelota.
        
        Args:
            ball_world_pos: np.array([X, Y]) posición en cm, o None si
                           la pelota no se detecta.
                           
        Returns:
            GameState actualizado
        """
        if self.state.game_over:
            return self.state

        # Gestionar celebración de hoyo embocado
        if self._celebrating:
            elapsed = time.time() - self._hole_in_time
            if elapsed > self._celebration_duration:
                self._celebrating = False
                self._advance_level()
            else:
                remaining = self._celebration_duration - elapsed
                self.state.status = f"¡EMBOCADA! Siguiente nivel en {remaining:.0f}s"
            return self.state

        if ball_world_pos is None:
            self.state.status = "Pelota no detectada"
            return self.state

        self.state.ball_world_pos = ball_world_pos.copy()

        # Almacenar historial de posiciones
        self._last_positions.append(ball_world_pos.copy())
        if len(self._last_positions) > self._max_history:
            self._last_positions = self._last_positions[-self._max_history:]

        # Calcular distancia al hoyo
        if self.state.hole_position is not None:
            dist = np.linalg.norm(ball_world_pos - self.state.hole_position)
            self.state.distance_to_hole = dist
        else:
            dist = float('inf')

        # Comprobar si la pelota se está moviendo
        is_moving = self._check_movement()
        was_moving = self.state.is_ball_moving
        self.state.is_ball_moving = is_moving

        # Transición: moviendo → parada
        if was_moving and not is_moving:
            self.state.putts += 1
            print(f"[MiniGolf] Golpe #{self.state.putts} - "
                  f"Pelota parada en ({ball_world_pos[0]:.1f}, "
                  f"{ball_world_pos[1]:.1f}) cm. "
                  f"Distancia al hoyo: {dist:.1f} cm")

        # Comprobar hoyo embocado
        if dist < config.HOLE_IN_TOLERANCE_CM:
            if not is_moving or dist < config.HOLE_RADIUS_CM:
                self._hole_in()
                return self.state

        # Actualizar mensaje de estado
        if is_moving:
            self.state.status = f"Pelota en movimiento... Dist: {dist:.1f} cm"
        else:
            if self.state.putts == 0:
                self.state.status = (f"Nivel {self.state.level} - "
                                     f"¡Golpea la pelota!")
            elif self.state.putts >= config.MAX_PUTTS_PER_HOLE:
                self.state.status = "Máximo de golpes alcanzado"
                self.state.putts = config.MAX_PUTTS_PER_HOLE
                self._advance_level()
            else:
                self.state.status = (f"Golpes: {self.state.putts} | "
                                     f"Dist: {dist:.1f} cm")

        return self.state

    def _check_movement(self):
        """
        Comprueba si la pelota está en movimiento o parada.
        
        Returns:
            True si la pelota se está moviendo.
        """
        if len(self._last_positions) < config.BALL_STOPPED_FRAMES:
            return True  # No suficientes datos, asumir movimiento

        recent = self._last_positions[-config.BALL_STOPPED_FRAMES:]

        # Calcular la dispersión de las últimas posiciones
        positions = np.array(recent)
        max_displacement = 0

        for i in range(1, len(positions)):
            d = np.linalg.norm(positions[i] - positions[i - 1])
            max_displacement = max(max_displacement, d)

        if max_displacement < config.BALL_STOPPED_THRESHOLD_CM:
            self._stopped_frame_count += 1
        else:
            self._stopped_frame_count = 0

        return self._stopped_frame_count < config.BALL_STOPPED_FRAMES

    def _hole_in(self):
        """Procesa una pelota embocada."""
        self.state.is_hole_in = True
        self._celebrating = True
        self._hole_in_time = time.time()

        # Puntuación: menos golpes = mejor
        if self.state.putts == 0:
            self.state.putts = 1  # Al menos un golpe

        score = self.state.putts
        self.state.total_score += score

        self.state.status = (f"¡¡¡EMBOCADA en {self.state.putts} golpe"
                             f"{'s' if self.state.putts != 1 else ''}!!!")

        print(f"\n[MiniGolf] *** ¡EMBOCADA! *** "
              f"Nivel {self.state.level} completado en "
              f"{self.state.putts} golpe(s).")
        print(f"[MiniGolf] Puntuación nivel: {score} | "
              f"Total: {self.state.total_score}\n")

    def _advance_level(self):
        """Avanza al siguiente nivel."""
        self.state.level += 1
        self.state.putts = 0
        self.state.is_hole_in = False
        self._stopped_frame_count = 0
        self._last_positions = []

        if self.state.level > len(config.HOLE_POSITIONS):
            # Se completaron todos los niveles
            self.state.game_over = True
            self.state.status = (f"¡¡PARTIDA COMPLETADA!! "
                                 f"Puntuación total: {self.state.total_score}")
            print(f"\n[MiniGolf] =================================")
            print(f"[MiniGolf] ¡PARTIDA COMPLETADA!")
            print(f"[MiniGolf] Puntuación final: {self.state.total_score}")
            print(f"[MiniGolf] =================================\n")
        else:
            self._set_hole_for_level(self.state.level)
            self.state.status = f"Nivel {self.state.level} - ¡A jugar!"
            self._level_start_time = time.time()
            print(f"[MiniGolf] Avanzando al nivel {self.state.level}")

    def restart_level(self):
        """Reinicia el nivel actual."""
        self.state.putts = 0
        self.state.is_hole_in = False
        self._stopped_frame_count = 0
        self._last_positions = []
        self._celebrating = False
        self.state.status = f"Nivel {self.state.level} reiniciado"
        print(f"[MiniGolf] Nivel {self.state.level} reiniciado")

    def restart_game(self):
        """Reinicia toda la partida."""
        self.start_game()

    def get_game_state_dict(self):
        """Devuelve el estado como diccionario."""
        return self.state.to_dict()

    # --------------------------------------------------------
    # Colisiones con obstáculos (extensión)
    # --------------------------------------------------------
    def check_obstacle_collision(self, ball_pos):
        """
        Comprueba si la pelota colisiona con algún obstáculo.
        
        Args:
            ball_pos: np.array([X, Y]) en cm
            
        Returns:
            True si hay colisión, (ox, oy, orad) del obstáculo, o
            False, None si no hay colisión.
        """
        for (ox, oy, orad) in self.state.obstacles:
            obstacle_center = np.array([ox, oy])
            dist = np.linalg.norm(ball_pos - obstacle_center)
            combined_radius = orad + config.BALL_REAL_RADIUS_CM
            if dist < combined_radius:
                return True, (ox, oy, orad)
        return False, None

    def is_ball_in_bounds(self, ball_pos):
        """
        Comprueba si la pelota está dentro del área de juego.
        
        Args:
            ball_pos: np.array([X, Y]) en cm
            
        Returns:
            True si está dentro.
        """
        margin = config.BALL_REAL_RADIUS_CM
        return (margin <= ball_pos[0] <= config.PLAY_AREA_WIDTH - margin and
                margin <= ball_pos[1] <= config.PLAY_AREA_HEIGHT - margin)
