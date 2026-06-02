"""Visor AR adaptado para portería de penaltis."""

import cv2
import numpy as np
import game_config as config


class PenaltisARViewer:
    def __init__(self, calibrator):
        self.calibrator = calibrator
        self.trail_image_points = []
        self._max_trail = 200
        self.goal_posts = None

    def set_goal_posts(self, p1_world, p2_world):
        self.goal_posts = (np.array(p1_world), np.array(p2_world))

    def draw(self, frame, game_state=None, ball_detection=None):
        output = frame.copy()

        if not self.calibrator.is_calibrated:
            cv2.putText(output, "No calibrado", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            return output

        self._draw_play_area(output)

        if self.goal_posts is not None:
            self._draw_goal(output)

        self._draw_trail(output)

        if game_state is not None:
            self._draw_game_info(output, game_state)

        if ball_detection is not None:
            self._draw_ball_detection(output, ball_detection)

        return output

    def _draw_play_area(self, frame):
        corners_world = config.WORLD_CORNERS
        corners_image = []
        for corner in corners_world:
            pt = self.calibrator.world_to_image(corner)
            if pt is not None:
                corners_image.append(pt.astype(int))
        if len(corners_image) == 4:
            pts = np.array(corners_image, dtype=np.int32)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

    def _draw_goal(self, frame):
        p1_w, p2_w = self.goal_posts
        p1_i = self.calibrator.world_to_image(p1_w)
        p2_i = self.calibrator.world_to_image(p2_w)
        if p1_i is None or p2_i is None:
            return

        p1 = tuple(p1_i.astype(int))
        p2 = tuple(p2_i.astype(int))

        # Postes
        cv2.line(frame, p1, (p1[0], p1[1] - 40), (255, 255, 255), 5)
        cv2.line(frame, p2, (p2[0], p2[1] - 40), (255, 255, 255), 5)

        # Travesaño
        cv2.line(frame, (p1[0], p1[1] - 40), (p2[0], p2[1] - 40), (255, 255, 255), 5)

        # Red
        for i in range(1, 5):
            y = p1[1] - int(40 * i / 5)
            cv2.line(frame, (p1[0], y), (p2[0], y), (200, 200, 255), 1)

        # Área sombreada (área de gol)
        goal_vec = p2_w - p1_w
        perp = np.array([-goal_vec[1], goal_vec[0]])
        norm = np.linalg.norm(perp)
        if norm > 0:
            perp = perp / norm * config.GOAL_DEPTH_CM
            p3_w = p2_w + perp
            p4_w = p1_w + perp
            p3_i = self.calibrator.world_to_image(p3_w)
            p4_i = self.calibrator.world_to_image(p4_w)
            if p3_i is not None and p4_i is not None:
                pts = np.array([p1_i, p2_i, p3_i, p4_i], dtype=np.int32)
                overlay = frame.copy()
                cv2.fillPoly(overlay, [pts], (0, 0, 255))
                cv2.addWeighted(overlay, 0.15, frame, 0.85, 0, frame)

        # Texto
        mid_x = (p1[0] + p2[0]) // 2
        mid_y = (p1[1] + p2[1]) // 2 - 55
        cv2.putText(frame, "PORTERIA", (mid_x - 50, mid_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    def _draw_trail(self, frame):
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
        cx, cy, radius = detection
        cv2.circle(frame, (cx, cy), int(radius), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)
        cs = 10
        cv2.line(frame, (cx - cs, cy), (cx + cs, cy), (0, 255, 0), 1)
        cv2.line(frame, (cx, cy - cs), (cx, cy + cs), (0, 255, 0), 1)

    def _draw_game_info(self, frame, game_state):
        h, w = frame.shape[:2]
        y = 30
        overlay = frame.copy()
        cv2.rectangle(overlay, (w - 260, 0), (w, 130), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

        shots = game_state.get('shots', 0)
        goals = game_state.get('goals', 0)
        status = game_state.get('status', '')

        cv2.putText(frame, f"Tiros: {shots}", (w - 250, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        y += 30
        cv2.putText(frame, f"Goles: {goals}", (w - 250, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        y += 30
        pos = game_state.get('ball_world_pos')
        if pos is not None:
            cv2.putText(frame, f"Pos: ({pos[0]:.1f},{pos[1]:.1f})",
                        (w - 250, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        if status:
            cv2.putText(frame, status, (20, h - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

    def add_trail_point(self, image_point):
        if image_point is not None:
            self.trail_image_points.append(np.array(image_point, dtype=np.float64))
            if len(self.trail_image_points) > self._max_trail:
                self.trail_image_points = self.trail_image_points[-self._max_trail:]

    def clear_trail(self):
        self.trail_image_points = []