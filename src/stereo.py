"""Reconstrucción 3D estéreo para el globo usando dos cámaras.

Convenciones:
- La calibración estéreo devuelve puntos en el sistema de la cámara izquierda.
- Opcionalmente se carga `calibration/world_transform.npz` para transformar esos
  puntos al sistema de mundo del juego: X_world = R_world_from_left @ X_left + t.
"""

import cv2
import numpy as np


class Stereo3DReconstructor:
    """Reconstruye puntos 3D a partir de dos vistas sincronizadas."""

    def __init__(self):
        self.K_left = None
        self.K_right = None
        self.dist_left = np.zeros((5, 1), dtype=np.float64)
        self.dist_right = np.zeros((5, 1), dtype=np.float64)
        self.R_right_from_left = None
        self.T_right_from_left = None
        self.F = None
        self.E = None
        self.image_size = None
        self.rms_left = None
        self.rms_right = None
        self.rms_stereo = None

        self.P_left = None
        self.P_right = None

        self.R_world_from_left = None
        self.t_world_from_left = None
        self.world_rms_cm = None

        self._calibrated = False

    @property
    def is_calibrated(self):
        return self._calibrated

    @property
    def has_world_transform(self):
        return self.R_world_from_left is not None and self.t_world_from_left is not None

    def load_calibration(self, path):
        """Carga calibración estéreo desde NPZ.

        Matrices de proyección en coordenadas de cámara izquierda:
            P_left  = K_left @ [I | 0]
            P_right = K_right @ [R | T]
        """
        try:
            with np.load(path) as data:
                self.K_left = data["K_l"].astype(np.float64)
                self.dist_left = data["dist_l"].astype(np.float64)
                self.K_right = data["K_r"].astype(np.float64)
                self.dist_right = data["dist_r"].astype(np.float64)
                self.R_right_from_left = data["R"].astype(np.float64)
                self.T_right_from_left = data["T"].astype(np.float64)
                self.E = data["E"].astype(np.float64) if "E" in data else None
                self.F = data["F"].astype(np.float64) if "F" in data else None
                self.image_size = (
                    tuple(data["image_size"].tolist()) if "image_size" in data else None
                )
                self.rms_left = float(data["rms_left"]) if "rms_left" in data else None
                self.rms_right = (
                    float(data["rms_right"]) if "rms_right" in data else None
                )
                self.rms_stereo = (
                    float(data["rms_stereo"]) if "rms_stereo" in data else None
                )

            self.P_left = self.K_left @ np.hstack(
                [np.eye(3, dtype=np.float64), np.zeros((3, 1), dtype=np.float64)]
            )
            self.P_right = self.K_right @ np.hstack(
                [self.R_right_from_left, self.T_right_from_left.reshape(3, 1)]
            )
            self._calibrated = True
            return True
        except Exception as e:
            print(f"[STEREO] No se pudo cargar calibración desde {path}: {e}")
            return False

    def load_world_transform(self, path):
        """Carga la transformación cámara izquierda → mundo del juego."""
        try:
            with np.load(path) as data:
                self.R_world_from_left = data["R_world_from_left"].astype(np.float64)
                self.t_world_from_left = (
                    data["t_world_from_left"].astype(np.float64).reshape(3)
                )
                self.world_rms_cm = (
                    float(data["world_rms_cm"]) if "world_rms_cm" in data else None
                )
            return True
        except Exception as e:
            print(
                f"[STEREO] No se pudo cargar transformación de mundo desde {path}: {e}"
            )
            return False

    def calibration_summary(self):
        baseline = None
        if self.T_right_from_left is not None:
            baseline = float(np.linalg.norm(self.T_right_from_left))
        return {
            "image_size": self.image_size,
            "baseline_cm": baseline,
            "rms_left": self.rms_left,
            "rms_right": self.rms_right,
            "rms_stereo": self.rms_stereo,
            "world_rms_cm": self.world_rms_cm,
            "has_world_transform": self.has_world_transform,
        }

    def epipolar_error_px(self, point_left, point_right):
        """Distancia simétrica aproximada punto-línea epipolar en píxeles."""
        if self.F is None:
            return None
        x_l = np.array(
            [float(point_left[0]), float(point_left[1]), 1.0], dtype=np.float64
        )
        x_r = np.array(
            [float(point_right[0]), float(point_right[1]), 1.0], dtype=np.float64
        )
        line_r = self.F @ x_l
        line_l = self.F.T @ x_r

        denom_r = np.hypot(line_r[0], line_r[1])
        denom_l = np.hypot(line_l[0], line_l[1])
        if denom_r < 1e-12 or denom_l < 1e-12:
            return None

        dist_r = abs(float(x_r @ line_r)) / denom_r
        dist_l = abs(float(x_l @ line_l)) / denom_l
        return 0.5 * (dist_l + dist_r)

    def _undistorted_pixel_columns(self, point_left, point_right):
        assert self.K_left is not None
        assert self.K_right is not None
        assert self.dist_left is not None
        assert self.dist_right is not None
        u_l, v_l = float(point_left[0]), float(point_left[1])
        u_r, v_r = float(point_right[0]), float(point_right[1])

        pts_l = cv2.undistortPoints(
            np.array([[[u_l, v_l]]], dtype=np.float64),
            self.K_left,
            self.dist_left,
            P=self.K_left,
        )
        pts_r = cv2.undistortPoints(
            np.array([[[u_r, v_r]]], dtype=np.float64),
            self.K_right,
            self.dist_right,
            P=self.K_right,
        )
        pts_l = np.array([[pts_l[0, 0, 0]], [pts_l[0, 0, 1]]], dtype=np.float64)
        pts_r = np.array([[pts_r[0, 0, 0]], [pts_r[0, 0, 1]]], dtype=np.float64)
        return pts_l, pts_r

    def triangulate_camera_left(self, point_left, point_right):
        """Triangula un punto 3D en el sistema de coordenadas de la cámara izquierda."""
        if not self._calibrated or self.P_left is None or self.P_right is None:
            return None

        pts_l, pts_r = self._undistorted_pixel_columns(point_left, point_right)
        point_4d = cv2.triangulatePoints(self.P_left, self.P_right, pts_l, pts_r)
        w = point_4d[3, 0]
        if abs(w) < 1e-10:
            return None

        point_3d = (point_4d[:3, 0] / w).astype(np.float64)
        if not np.isfinite(point_3d).all():
            return None
        if abs(point_3d[2]) > 1000.0:
            return None
        return point_3d

    def camera_left_to_world(self, point_left_camera):
        if not self.has_world_transform:
            return None
        assert self.R_world_from_left is not None
        assert self.t_world_from_left is not None
        return (
            self.R_world_from_left
            @ np.asarray(point_left_camera, dtype=np.float64).reshape(3)
            + self.t_world_from_left
        )

    def world_to_camera_left(self, point_world):
        if not self.has_world_transform:
            return None
        assert self.R_world_from_left is not None
        assert self.t_world_from_left is not None
        point_world = np.asarray(point_world, dtype=np.float64).reshape(3)
        return self.R_world_from_left.T @ (point_world - self.t_world_from_left)

    def triangulate_world(self, point_left, point_right):
        """Triangula y devuelve coordenadas en el sistema de mundo si está calibrado."""
        point_left_camera = self.triangulate_camera_left(point_left, point_right)
        if point_left_camera is None:
            return None
        if not self.has_world_transform:
            return point_left_camera
        return self.camera_left_to_world(point_left_camera)

    def triangulate(self, point_left, point_right):
        """Alias compatible: devuelve mundo si existe transformación; si no, cámara izquierda."""
        return self.triangulate_world(point_left, point_right)

    def project_world_to_left(self, point_world):
        """Proyecta un punto del mundo en la imagen izquierda."""
        if self.K_left is None or self.dist_left is None:
            return None
        point_camera = self.world_to_camera_left(point_world)
        if point_camera is None or point_camera[2] <= 1e-6:
            return None
        img_pts, _ = cv2.projectPoints(
            point_camera.reshape(1, 1, 3),
            np.zeros((3, 1), dtype=np.float64),
            np.zeros((3, 1), dtype=np.float64),
            self.K_left,
            self.dist_left,
        )
        u, v = img_pts[0, 0]
        if not np.isfinite([u, v]).all():
            return None
        return int(round(u)), int(round(v))
