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

        self.F_refined = None

        self._rectification_ready = False
        self.map_l1 = self.map_l2 = None
        self.map_r1 = self.map_r2 = None
        self.Q = None

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
        F = self.F_refined if self.F_refined is not None else self.F
        if F is None:
            return None
        x_l = np.array(
            [float(point_left[0]), float(point_left[1]), 1.0], dtype=np.float64
        )
        x_r = np.array(
            [float(point_right[0]), float(point_right[1]), 1.0], dtype=np.float64
        )
        line_r = F @ x_l
        line_l = F.T @ x_r

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
        if point_3d[2] <= 0.0:
            return None
        if point_3d[2] > 1000.0:
            return None
        return point_3d

    def camera_left_to_world(self, point_left_camera):
        if not self.has_world_transform:
            return None
        point_left_camera = np.asarray(point_left_camera, dtype=np.float64).ravel()
        if point_left_camera.shape[0] != 3 or not np.isfinite(point_left_camera).all():
            return None
        assert self.R_world_from_left is not None
        assert self.t_world_from_left is not None
        return (
            self.R_world_from_left @ point_left_camera + self.t_world_from_left
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
        if self.K_left is None or self.dist_left is None:
            return None
        if not self.has_world_transform:
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

    def project_world_to_right(self, point_world):
        """Proyecta un punto del mundo en la imagen derecha."""
        if self.K_right is None or self.dist_right is None:
            return None
        if not self.has_world_transform:
            return None
        point_left_camera = self.world_to_camera_left(point_world)
        if point_left_camera is None or point_left_camera[2] <= 1e-6:
            return None
        point_right_camera = (
            self.R_right_from_left @ point_left_camera.reshape(3)
            + self.T_right_from_left.ravel()
        )
        if point_right_camera[2] <= 1e-6:
            return None
        img_pts, _ = cv2.projectPoints(
            point_right_camera.reshape(1, 1, 3),
            np.zeros((3, 1), dtype=np.float64),
            np.zeros((3, 1), dtype=np.float64),
            self.K_right,
            self.dist_right,
        )
        u, v = img_pts[0, 0]
        if not np.isfinite([u, v]).all():
            return None
        return int(round(u)), int(round(v))

    def init_rectification(self):
        """Precomputa mapas de rectificacion estéreo para SGBM."""
        if not self._calibrated or self.image_size is None:
            return False
        R1, R2, P1, P2, self.Q, _, _ = cv2.stereoRectify(
            self.K_left, self.dist_left,
            self.K_right, self.dist_right,
            self.image_size,
            self.R_right_from_left, self.T_right_from_left,
            alpha=0,
        )
        self.map_l1, self.map_l2 = cv2.initUndistortRectifyMap(
            self.K_left, self.dist_left, R1, P1, self.image_size, cv2.CV_32FC1
        )
        self.map_r1, self.map_r2 = cv2.initUndistortRectifyMap(
            self.K_right, self.dist_right, R2, P2, self.image_size, cv2.CV_32FC1
        )
        self._rectification_ready = True
        return True

    def rectify_pair(self, img_l, img_r):
        """Rectifica un par estéreo para matching denso."""
        if not self._rectification_ready:
            return None, None
        rect_l = cv2.remap(img_l, self.map_l1, self.map_l2, cv2.INTER_LINEAR)
        rect_r = cv2.remap(img_r, self.map_r1, self.map_r2, cv2.INTER_LINEAR)
        return rect_l, rect_r

    def compute_disparity_colormap(self, img_l, img_r):
        """Disparidad densa (SGBM) convertida a colormap JET."""
        rect_l, rect_r = self.rectify_pair(img_l, img_r)
        if rect_l is None:
            return None
        gray_l = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(rect_r, cv2.COLOR_BGR2GRAY)
        stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=128,
            blockSize=11,
            P1=8 * 1 * 11 ** 2,
            P2=32 * 1 * 11 ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
        )
        disparity = stereo.compute(gray_l, gray_r).astype(np.float32) / 16.0
        mask = disparity > 0
        disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        disp_vis[~mask] = 0
        colormap = cv2.applyColorMap(disp_vis.astype(np.uint8), cv2.COLORMAP_JET)
        return colormap

    def pixel_to_floor_world(self, u, v):
        """Interseccion rayo-plano suelo: pixel (u,v) → X_world sobre Z=0."""
        if not self.has_world_transform or self.K_left is None:
            return None
        K_inv = np.linalg.inv(self.K_left)
        ray = K_inv @ np.array([float(u), float(v), 1.0], dtype=np.float64)
        normal = self.R_world_from_left[2, :]
        d = self.t_world_from_left[2]
        denom = float(normal @ ray)
        if abs(denom) < 1e-10:
            return None
        lam = -d / denom
        if lam <= 0:
            return None
        X_cam = lam * ray
        X_world = self.R_world_from_left @ X_cam + self.t_world_from_left
        return X_world

    def refine_fundamental_from_floor(self, img_l_path, img_r_path):
        """Refina F usando esquinas ChArUco en la imagen de suelo (RANSAC)."""
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters()
        try:
            detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        except AttributeError:
            detector = None

        board = cv2.aruco.CharucoBoard((5, 7), 4.0, 3.0, aruco_dict)

        pts_l = []
        pts_r = []
        for path, side in [(img_l_path, 'L'), (img_r_path, 'R')]:
            img = cv2.imread(path)
            if img is None:
                print(f"[F-REFINE] No se pudo leer {path}")
                return False
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            if detector is not None:
                corners, ids, _ = detector.detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            if ids is None:
                print(f"[F-REFINE] Sin marcadores en {path}")
                return False
            ret, ch, ch_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
            if not ret or ch_ids is None:
                print(f"[F-REFINE] Sin esquinas ChArUco en {path}")
                return False
            if side == 'L':
                pts_l = [(ch[i, 0, 0], ch[i, 0, 1]) for i in range(len(ch))]
                ids_l = ch_ids.flatten()
            else:
                pts_r = [(ch[i, 0, 0], ch[i, 0, 1]) for i in range(len(ch))]
                ids_r = ch_ids.flatten()

        common = sorted(set(ids_l.tolist()) & set(ids_r.tolist()))
        if len(common) < 10:
            print(f"[F-REFINE] Solo {len(common)} esquinas comunes, necesarias >=10")
            return False

        idx_l = {cid: i for i, cid in enumerate(ids_l.tolist())}
        idx_r = {cid: i for i, cid in enumerate(ids_r.tolist())}
        matched_l = np.array([pts_l[idx_l[c]] for c in common], dtype=np.float64)
        matched_r = np.array([pts_r[idx_r[c]] for c in common], dtype=np.float64)

        F_ref, mask = cv2.findFundamentalMat(matched_l, matched_r, cv2.FM_RANSAC, 2.0, 0.99)
        if F_ref is None:
            print("[F-REFINE] findFundamentalMat fallo")
            return False

        self.F_refined = F_ref
        inliers = int(mask.sum()) if mask is not None else len(common)
        print(f"[F-REFINE] F refinado desde {len(common)} esquinas, {inliers} inliers RANSAC")
        return True

    def snap_to_epipolar(self, pt_left, pt_right):
        """Proyecta pt_right a la linea epipolar de pt_left y viceversa, devuelve par corregido."""
        F = self.F_refined if self.F_refined is not None else self.F
        if F is None:
            return pt_left, pt_right
        x_l = np.array([pt_left[0], pt_left[1], 1.0], dtype=np.float64)
        x_r = np.array([pt_right[0], pt_right[1], 1.0], dtype=np.float64)
        line_r = F @ x_l
        line_l = F.T @ x_r
        a_r, b_r, c_r = line_r[0], line_r[1], line_r[2]
        a_l, b_l, c_l = line_l[0], line_l[1], line_l[2]
        den_r = a_r * a_r + b_r * b_r
        den_l = a_l * a_l + b_l * b_l
        if den_r < 1e-12 or den_l < 1e-12:
            return pt_left, pt_right
        u_r = (b_r * (b_r * x_r[0] - a_r * x_r[1]) - a_r * c_r) / den_r
        v_r = (a_r * (-b_r * x_r[0] + a_r * x_r[1]) - b_r * c_r) / den_r
        u_l = (b_l * (b_l * x_l[0] - a_l * x_l[1]) - a_l * c_l) / den_l
        v_l = (a_l * (-b_l * x_l[0] + a_l * x_l[1]) - b_l * c_l) / den_l
        return (u_l, v_l), (u_r, v_r)
