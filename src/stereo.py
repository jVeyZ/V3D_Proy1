"""Reconstruccion 3D estereo para la pelota usando dos camaras."""

import cv2
import numpy as np


class Stereo3DReconstructor:
    """Reconstruye puntos 3D a partir de dos vistas sincronizadas."""

    def __init__(self):
        self.K_left = None
        self.K_right = None
        self.dist_left = np.zeros((5, 1), dtype=np.float64)
        self.dist_right = np.zeros((5, 1), dtype=np.float64)

        self.P_left = None
        self.P_right = None

        self._calibrated = False

    @property
    def is_calibrated(self):
        return self._calibrated

    def load_calibration(self, path):
        """Load stereo calibration (K, dist, R, T) from NPZ file.

        Sets projection matrices in left-camera coordinates:
            P_left  = K_left @ [I | 0]
            P_right = K_right @ [R | T]
        """
        try:
            with np.load(path) as data:
                self.K_left = data["K_l"]
                self.dist_left = data["dist_l"]
                self.K_right = data["K_r"]
                self.dist_right = data["dist_r"]
                R = data["R"]
                T = data["T"]
            self.P_left = self.K_left @ np.hstack(
                [np.eye(3, dtype=np.float64), np.zeros((3, 1), dtype=np.float64)]
            )
            self.P_right = self.K_right @ np.hstack([R, T])
            self._calibrated = True
            return True
        except Exception as e:
            print(f"[STEREO] No se pudo cargar calibración desde {path}: {e}")
            return False

    def triangulate(self, point_left, point_right):
        """Triangula un punto 3D en el sistema de coordenadas de la cámara izquierda."""
        if not self._calibrated or self.P_left is None or self.P_right is None:
            return None

        u_l, v_l = float(point_left[0]), float(point_left[1])
        u_r, v_r = float(point_right[0]), float(point_right[1])

        if self.dist_left is not None and self.K_left is not None:
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
            pts_l = np.array(
                [[pts_l[0, 0, 0]], [pts_l[0, 0, 1]]], dtype=np.float64
            )
            pts_r = np.array(
                [[pts_r[0, 0, 0]], [pts_r[0, 0, 1]]], dtype=np.float64
            )
        else:
            pts_l = np.array([[u_l], [v_l]], dtype=np.float64)
            pts_r = np.array([[u_r], [v_r]], dtype=np.float64)

        point_4d = cv2.triangulatePoints(self.P_left, self.P_right, pts_l, pts_r)
        w = point_4d[3, 0]
        if abs(w) < 1e-10:
            return None

        point_3d = (point_4d[:3, 0] / w).astype(np.float64)

        if not np.isfinite(point_3d).all():
            return None
        if abs(point_3d[2]) > 500.0:
            return None

        return point_3d
