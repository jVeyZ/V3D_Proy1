"""Reconstruccion 3D estereo para la pelota usando dos camaras."""

import cv2
import numpy as np
import src.game_config as config 


class Stereo3DReconstructor:
    """Reconstruye puntos 3D a partir de dos vistas sincronizadas."""

    def __init__(self, marker_ids_order=(0, 1, 2, 3), fov_deg=None):
        self.marker_ids_order = tuple(marker_ids_order)
        self.fov_deg = float(
            fov_deg if fov_deg is not None else config.STEREO_DEFAULT_FOV_DEG
        )

        self.K_left = None
        self.K_right = None
        self.dist_left = np.zeros((5, 1), dtype=np.float64)
        self.dist_right = np.zeros((5, 1), dtype=np.float64)

        self.P_left = None
        self.P_right = None

        self.rvec_left = None
        self.tvec_left = None
        self.rvec_right = None
        self.tvec_right = None

        self._calibrated = False

    @property
    def is_calibrated(self):
        return self._calibrated

    def reset(self):
        self.P_left = None
        self.P_right = None
        self.rvec_left = None
        self.tvec_left = None
        self.rvec_right = None
        self.tvec_right = None
        self._calibrated = False

    def load_calibration(self, path):
        """Load calibrated intrinsics (K, dist) from NPZ file.

        P matrices are NOT set here — they must be computed from the
        current ArUco board pose via update_pose() so that triangulation
        happens in world (board) coordinates.
        """
        try:
            with np.load(path) as data:
                self.K_left = data["K_l"]
                self.dist_left = data["dist_l"]
                self.K_right = data["K_r"]
                self.dist_right = data["dist_r"]
            self._calibrated = True
            return True
        except Exception as e:
            print(f"[STEREO] No se pudo cargar calibración desde {path}: {e}")
            return False

    def update_pose(self, img_pts_left, img_pts_right, frame_shape):
        """Compute P matrices in world (board) coordinates via solvePnP.

        Uses stored intrinsics (K, dist) — from NPZ if loaded, or
        FOV-estimated otherwise.  Returns True on success.
        """
        obj_pts = self._world_object_points_3d()
        h, w = frame_shape

        if self.K_left is None:
            self.K_left = self._estimate_camera_matrix(w, h, self.fov_deg)
        if self.K_right is None:
            self.K_right = self._estimate_camera_matrix(w, h, self.fov_deg)

        ok_l, rvec_l, tvec_l = cv2.solvePnP(
            obj_pts, img_pts_left, self.K_left, self.dist_left,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        ok_r, rvec_r, tvec_r = cv2.solvePnP(
            obj_pts, img_pts_right, self.K_right, self.dist_right,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

        if not ok_l or not ok_r:
            return False

        R_l, _ = cv2.Rodrigues(rvec_l)
        R_r, _ = cv2.Rodrigues(rvec_r)

        self.P_left = self.K_left @ np.hstack([R_l, tvec_l])
        self.P_right = self.K_right @ np.hstack([R_r, tvec_r])

        self.rvec_left, self.tvec_left = rvec_l, tvec_l
        self.rvec_right, self.tvec_right = rvec_r, tvec_r
        self._calibrated = True
        return True

    def calibrate_from_aruco(self, frame_left, frame_right):
        """Deprecated — use update_pose() instead."""
        img_pts_left = self._detect_ordered_marker_centers(frame_left)
        img_pts_right = self._detect_ordered_marker_centers(frame_right)
        if img_pts_left is None or img_pts_right is None:
            return False
        return self.update_pose(img_pts_left, img_pts_right, frame_left.shape[:2])

    def triangulate(self, point_left, point_right):
        """Triangula un punto 3D en centimetros en el sistema mundo del tablero."""
        if not self._calibrated or self.P_left is None or self.P_right is None:
            return None

        u_l, v_l = float(point_left[0]), float(point_left[1])
        u_r, v_r = float(point_right[0]), float(point_right[1])

        if self.dist_left is not None and self.K_left is not None:
            pts_l = cv2.undistortPoints(
                np.array([[[u_l, v_l]]], dtype=np.float64),
                self.K_left,
                self.dist_left,
                P=self.K_left
            )
            pts_r = cv2.undistortPoints(
                np.array([[[u_r, v_r]]], dtype=np.float64),
                self.K_right,
                self.dist_right,
                P=self.K_right
            )
            pts_l = np.array([[pts_l[0, 0, 0]], [pts_l[0, 0, 1]]], dtype=np.float64)
            pts_r = np.array([[pts_r[0, 0, 0]], [pts_r[0, 0, 1]]], dtype=np.float64)
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
        if abs(point_3d[2]) > 200.0:
            return None

        return point_3d

    def _world_object_points_3d(self):
        """Devuelve las 4 esquinas del tablero como puntos 3D (Z=0)."""
        world_2d = config.WORLD_CORNERS
        obj_pts = []
        for i in range(len(self.marker_ids_order)):
            x, y = world_2d[i]
            obj_pts.append([float(x), float(y), 0.0])
        return np.array(obj_pts, dtype=np.float64)

    @staticmethod
    def _estimate_camera_matrix(width, height, fov_deg):
        """Aproxima matriz intrinseca a partir de tamano y FOV horizontal."""
        fov_rad = np.deg2rad(float(fov_deg))
        fx = (0.5 * width) / np.tan(0.5 * fov_rad)
        fy = fx
        cx = width / 2.0
        cy = height / 2.0
        return np.array(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64
        )

    def _detect_ordered_marker_centers(self, frame):
        """Detecta centros ArUco en el orden TL, TR, BR, BL por ID."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters()

        try:
            detector = cv2.aruco.ArucoDetector(aruco_dict, params)
            corners, ids, _ = detector.detectMarkers(gray)
        except AttributeError:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, aruco_dict, parameters=params
            )

        if ids is None:
            return None

        id_to_center = {}
        for i, marker_id in enumerate(ids.flatten()):
            center = corners[i][0].mean(axis=0)
            id_to_center[int(marker_id)] = center

        ordered = []
        for marker_id in self.marker_ids_order:
            if marker_id not in id_to_center:
                return None
            ordered.append(id_to_center[marker_id])

        return np.array(ordered, dtype=np.float64)
