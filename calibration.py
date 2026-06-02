"""
calibration.py - Calibración de la cámara mediante homografía.
"""

import cv2
import numpy as np
import game_config as config  # ← CAMBIO: alias para no tocar todo el código


class HomographyCalibrator:
    """Calcula y gestiona la homografía imagen ↔ plano de trabajo."""

    def __init__(self):
        self.H = None
        self.H_inv = None
        self.image_corners = None
        self.world_corners = config.WORLD_CORNERS.copy() 
        self._click_points = []
        self._calibration_done = False

    @property
    def is_calibrated(self):
        return self._calibration_done

    def calibrate_manual(self, frame):
        self._click_points = []
        window_name = "Calibracion: clic en 4 esquinas (TL, TR, BR, BL)"
        instructions = [
            "1. Superior-Izquierda",
            "2. Superior-Derecha",
            "3. Inferior-Derecha",
            "4. Inferior-Izquierda",
        ]

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(window_name, self._mouse_callback_calibration)

        while len(self._click_points) < 4:
            display = frame.copy()

            for i, pt in enumerate(self._click_points):
                cv2.circle(display, pt, 8, (0, 255, 0), -1)
                cv2.putText(display, str(i + 1), (pt[0] + 10, pt[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                if i > 0:
                    cv2.line(
                        display, self._click_points[i - 1], pt, (0, 255, 0), 2)

            idx = len(self._click_points)
            msg = f"Haz clic en la esquina {instructions[idx]}"
            cv2.putText(display, msg, (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(display, "Pulsa 'r' para reiniciar, 'ESC' para cancelar",
                        (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

            cv2.imshow(window_name, display)
            key = cv2.waitKey(30) & 0xFF
            if key == 27:
                cv2.destroyWindow(window_name)
                return False
            elif key == ord('r'):
                self._click_points = []

        display = frame.copy()
        pts = np.array(self._click_points, dtype=np.int32)
        cv2.polylines(display, [pts], True, (0, 255, 0), 2)
        for i, pt in enumerate(self._click_points):
            cv2.circle(display, pt, 8, (0, 255, 0), -1)
        cv2.putText(display, "Calibracion completada. Pulsa cualquier tecla.",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow(window_name, display)
        cv2.waitKey(0)
        cv2.destroyWindow(window_name)

        self.image_corners = np.array(self._click_points, dtype=np.float64)
        return self._compute_homography()

    def _mouse_callback_calibration(self, event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN and len(self._click_points) < 4:
            self._click_points.append((x, y))

    def calibrate_aruco(self, frame, marker_ids_order=(0, 1, 2, 3), quiet=False):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()

        try:
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
            corners, ids, _ = detector.detectMarkers(gray)
        except AttributeError:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, aruco_dict, parameters=parameters)

        if ids is None or len(ids) < 4:
            if not quiet:
                print(f"[ArUco] Solo se detectaron {0 if ids is None else len(ids)} "
                      f"marcadores. Se necesitan 4.")
            return False

        id_to_center = {}
        for i, marker_id in enumerate(ids.flatten()):
            center = corners[i][0].mean(axis=0)
            id_to_center[marker_id] = center

        image_pts = []
        for mid in marker_ids_order:
            if mid not in id_to_center:
                if not quiet:
                    print(f"[ArUco] No se encontró el marcador ID={mid}")
                return False
            image_pts.append(id_to_center[mid])

        self.image_corners = np.array(image_pts, dtype=np.float64)
        if not quiet:
            print(f"[ArUco] Esquinas detectadas: {self.image_corners}")
        return self._compute_homography(quiet=quiet)

    def _compute_homography(self, quiet=False):
        self.H, status = cv2.findHomography(
            self.image_corners, self.world_corners, cv2.RANSAC, 5.0
        )

        if self.H is None:
            if not quiet:
                print("[Calibración] Error al calcular la homografía.")
            self._calibration_done = False
            return False

        self.H_inv = np.linalg.inv(self.H)
        self._calibration_done = True

        error = self._reprojection_error()
        if not quiet:
            print(
                f"[Calibración] Homografía calculada. Error de reproyección: "
                f"{error:.3f} px"
            )
        return True

    def _reprojection_error(self):
        if self.H_inv is None:
            return float('inf')

        errors = []
        for img_pt, world_pt in zip(self.image_corners, self.world_corners):
            p = self.H_inv @ np.array([world_pt[0], world_pt[1], 1.0])
            p = p[:2] / p[2]
            errors.append(np.linalg.norm(p - img_pt))
        return np.mean(errors)

    def image_to_world(self, image_point):
        if not self._calibration_done:
            return None

        p = np.array([image_point[0], image_point[1], 1.0], dtype=np.float64)
        w = self.H @ p
        if abs(w[2]) < 1e-10:
            return None
        return np.array([w[0] / w[2], w[1] / w[2]])

    def world_to_image(self, world_point):
        if not self._calibration_done:
            return None

        p = np.array([world_point[0], world_point[1], 1.0], dtype=np.float64)
        q = self.H_inv @ p
        if abs(q[2]) < 1e-10:
            return None
        return np.array([q[0] / q[2], q[1] / q[2]])

    def set_homography_direct(self, H):
        self.H = H.copy()
        self.H_inv = np.linalg.inv(H)
        self._calibration_done = True