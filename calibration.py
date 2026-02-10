"""
calibration.py - Calibración de la cámara mediante homografía.

Permite obtener la transformación entre coordenadas de imagen (píxeles)
y coordenadas del plano de trabajo (cm), utilizando 4 puntos de correspondencia.

Modos:
  - Manual: el usuario hace clic en las 4 esquinas del área de juego.
  - Automático con ArUco: detecta 4 marcadores ArUco colocados en las esquinas.
"""

import cv2
import numpy as np
import config


class HomographyCalibrator:
    """Calcula y gestiona la homografía imagen ↔ plano de trabajo."""

    def __init__(self):
        self.H = None              # Homografía imagen → mundo
        self.H_inv = None          # Homografía mundo → imagen
        self.image_corners = None  # 4 esquinas en píxeles
        self.world_corners = config.WORLD_CORNERS.copy()
        self._click_points = []
        self._calibration_done = False

    @property
    def is_calibrated(self):
        return self._calibration_done

    # --------------------------------------------------------
    # Calibración manual (clic en 4 esquinas)
    # --------------------------------------------------------
    def calibrate_manual(self, frame):
        """
        Muestra la imagen y espera a que el usuario haga clic en las 4 esquinas
        del área de juego en el orden: sup-izq, sup-der, inf-der, inf-izq.
        """
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

            # Dibujar puntos ya seleccionados
            for i, pt in enumerate(self._click_points):
                cv2.circle(display, pt, 8, (0, 255, 0), -1)
                cv2.putText(display, str(i + 1), (pt[0] + 10, pt[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                if i > 0:
                    cv2.line(display, self._click_points[i - 1], pt, (0, 255, 0), 2)

            # Instrucciones
            idx = len(self._click_points)
            msg = f"Haz clic en la esquina {instructions[idx]}"
            cv2.putText(display, msg, (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(display, "Pulsa 'r' para reiniciar, 'ESC' para cancelar",
                        (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

            cv2.imshow(window_name, display)
            key = cv2.waitKey(30) & 0xFF
            if key == 27:  # ESC
                cv2.destroyWindow(window_name)
                return False
            elif key == ord('r'):
                self._click_points = []

        # Cerrar polígono
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

    def _mouse_callback_calibration(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(self._click_points) < 4:
            self._click_points.append((x, y))

    # --------------------------------------------------------
    # Calibración automática con ArUco
    # --------------------------------------------------------
    def calibrate_aruco(self, frame, marker_ids_order=(0, 1, 2, 3)):
        """
        Detecta 4 marcadores ArUco y usa sus centros como esquinas.
        marker_ids_order: IDs de los marcadores en orden TL, TR, BR, BL.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Diccionario ArUco
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()

        try:
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
            corners, ids, rejected = detector.detectMarkers(gray)
        except AttributeError:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, aruco_dict, parameters=parameters)

        if ids is None or len(ids) < 4:
            print(f"[ArUco] Solo se detectaron {0 if ids is None else len(ids)} "
                  f"marcadores. Se necesitan 4.")
            return False

        # Mapear ID → centro del marcador
        id_to_center = {}
        for i, marker_id in enumerate(ids.flatten()):
            center = corners[i][0].mean(axis=0)
            id_to_center[marker_id] = center

        # Verificar que están todos los IDs requeridos
        image_pts = []
        for mid in marker_ids_order:
            if mid not in id_to_center:
                print(f"[ArUco] No se encontró el marcador ID={mid}")
                return False
            image_pts.append(id_to_center[mid])

        self.image_corners = np.array(image_pts, dtype=np.float64)
        print(f"[ArUco] Esquinas detectadas: {self.image_corners}")
        return self._compute_homography()

    # --------------------------------------------------------
    # Cálculo de la homografía
    # --------------------------------------------------------
    def _compute_homography(self):
        """Calcula la homografía a partir de image_corners y world_corners."""
        self.H, status = cv2.findHomography(
            self.image_corners, self.world_corners, cv2.RANSAC, 5.0
        )

        if self.H is None:
            print("[Calibración] Error al calcular la homografía.")
            self._calibration_done = False
            return False

        self.H_inv = np.linalg.inv(self.H)
        self._calibration_done = True

        # Evaluar error de reproyección
        error = self._reprojection_error()
        print(f"[Calibración] Homografía calculada. Error de reproyección: "
              f"{error:.3f} px")
        return True

    def _reprojection_error(self):
        """Calcula el error de reproyección medio (px)."""
        if self.H_inv is None:
            return float('inf')

        errors = []
        for img_pt, world_pt in zip(self.image_corners, self.world_corners):
            # Proyectar punto mundo a imagen
            p = self.H_inv @ np.array([world_pt[0], world_pt[1], 1.0])
            p = p[:2] / p[2]
            errors.append(np.linalg.norm(p - img_pt))
        return np.mean(errors)

    # --------------------------------------------------------
    # Transformaciones
    # --------------------------------------------------------
    def image_to_world(self, image_point):
        """
        Transforma un punto de imagen (px) a coordenadas del plano (cm).
        
        Args:
            image_point: (u, v) en píxeles
        Returns:
            (X, Y) en cm sobre el plano de trabajo, o None si no está calibrado
        """
        if not self._calibration_done:
            return None

        p = np.array([image_point[0], image_point[1], 1.0], dtype=np.float64)
        w = self.H @ p
        if abs(w[2]) < 1e-10:
            return None
        return np.array([w[0] / w[2], w[1] / w[2]])

    def world_to_image(self, world_point):
        """
        Transforma un punto del plano (cm) a coordenadas de imagen (px).
        
        Args:
            world_point: (X, Y) en cm
        Returns:
            (u, v) en píxeles, o None si no está calibrado
        """
        if not self._calibration_done:
            return None

        p = np.array([world_point[0], world_point[1], 1.0], dtype=np.float64)
        q = self.H_inv @ p
        if abs(q[2]) < 1e-10:
            return None
        return np.array([q[0] / q[2], q[1] / q[2]])

    def set_homography_direct(self, H):
        """Establece la homografía directamente (para pruebas/demo)."""
        self.H = H.copy()
        self.H_inv = np.linalg.inv(H)
        self._calibration_done = True