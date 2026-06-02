"""Motor de juego de Penaltis de Coches."""

import time
import numpy as np
import game_config as config


class PenaltisState:
    def __init__(self):
        self.shots = 0
        self.goals = 0
        self.misses = 0
        self.status = "Esperando..."
        self.ball_world_pos = None
        self.ball_velocity = np.array([0.0, 0.0])
        self.goal_line = None
        self.is_goal = False
        self.game_over = False
        self.shooting = False

    def to_dict(self):
        return {
            'shots': self.shots,
            'goals': self.goals,
            'misses': self.misses,
            'status': self.status,
            'ball_world_pos': self.ball_world_pos,
            'is_goal': self.is_goal,
            'game_over': self.game_over,
        }


class PenaltisEngine:
    def __init__(self, max_shots=None):
        self.max_shots = max_shots or config.MAX_SHOTS
        self.state = PenaltisState()
        self._ball_history = []
        self._max_history = 10
        self._goal_cooldown = False
        self._goal_cooldown_time = 0
        self._goal_cooldown_duration = config.GOAL_COOLDOWN_SEC
        self._shot_active = False
        self._shot_start_pos = None

    def start_game(self):
        self.state = PenaltisState()
        self.state.status = "¡A jugar! Mueve el coche y dispara."
        self._ball_history = []
        self._goal_cooldown = False
        self._shot_active = False
        print("[Penaltis] Partida iniciada")

    def set_goal_line(self, p1, p2):
        self.state.goal_line = (np.array(p1), np.array(p2))

    def update(self, ball_world_pos):
        if self.state.game_over:
            return self.state

        now = time.time()

        # Cooldown tras gol
        if self._goal_cooldown:
            if now - self._goal_cooldown_time > self._goal_cooldown_duration:
                self._goal_cooldown = False
                self._advance_shot()
            else:
                remain = self._goal_cooldown_duration - (now - self._goal_cooldown_time)
                self.state.status = f"¡GOOOL! Nuevo tiro en {remain:.0f}s"
            return self.state

        if ball_world_pos is None:
            self.state.status = "Bola no detectada"
            return self.state

        self.state.ball_world_pos = ball_world_pos.copy()

        # Historial para velocidad
        self._ball_history.append((now, ball_world_pos.copy()))
        if len(self._ball_history) > self._max_history:
            self._ball_history.pop(0)

        if len(self._ball_history) >= 2:
            dt = self._ball_history[-1][0] - self._ball_history[-2][0]
            if dt > 0:
                self.state.ball_velocity = (
                    self._ball_history[-1][1] - self._ball_history[-2][1]
                ) / dt

        speed = np.linalg.norm(self.state.ball_velocity)

        # Detectar inicio de tiro
        if speed > 5.0 and not self._shot_active and not self._goal_cooldown:
            self._shot_active = True
            self._shot_start_pos = ball_world_pos.copy()
            self.state.shooting = True
            self.state.status = "¡Tiro en curso!"

        # Verificar gol
        if self._shot_active and self.state.goal_line is not None:
            if self._check_goal(ball_world_pos):
                self._register_goal()
                return self.state

        # Tiro terminado (bola parada)
        if self._shot_active and speed < 1.0:
            if not self.state.is_goal:
                self._register_miss()
            self._shot_active = False
            self.state.shooting = False

        # Límite de tiros
        if self.state.shots >= self.max_shots and not self._shot_active:
            self.state.game_over = True
            self.state.status = f"FIN. Goles: {self.state.goals}/{self.state.shots}"

        return self.state

    def _check_goal(self, ball_pos):
        if self.state.goal_line is None:
            return False
        p1, p2 = self.state.goal_line
        goal_vec = p2 - p1
        goal_len = np.linalg.norm(goal_vec)
        if goal_len < 1e-6:
            return False
        goal_dir = goal_vec / goal_len
        perp = np.array([-goal_dir[1], goal_dir[0]])

        local = ball_pos - p1
        along = np.dot(local, goal_dir)
        across = np.dot(local, perp)

        depth = config.GOAL_DEPTH_CM
        if along < -2.0 or along > goal_len + 2.0:
            return False
        if across < 0.5 or across > depth:
            return False
        if np.linalg.norm(self.state.ball_velocity) < 1.0:
            return False
        return True

    def _register_goal(self):
        self.state.goals += 1
        self.state.shots += 1
        self.state.is_goal = True
        self._goal_cooldown = True
        self._goal_cooldown_time = time.time()
        self.state.status = f"¡GOOOL! {self.state.goals}/{self.state.shots}"
        print(f"[Penaltis] ¡GOOOL! Goles: {self.state.goals}/{self.state.shots}")

    def _register_miss(self):
        self.state.shots += 1
        self.state.misses += 1
        self.state.status = f"Fallado. {self.state.shots}/{self.max_shots}"
        print(f"[Penaltis] Fallado. Goles: {self.state.goals}/{self.state.shots}")

    def _advance_shot(self):
        self.state.is_goal = False
        self._shot_active = False
        self.state.shooting = False
        if self.state.shots >= self.max_shots:
            self.state.game_over = True
            self.state.status = f"FIN. Goles: {self.state.goals}/{self.state.shots}"
        else:
            self.state.status = f"Preparando tiro {self.state.shots + 1}..."