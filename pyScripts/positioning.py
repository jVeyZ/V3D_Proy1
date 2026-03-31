"""Posicionamiento planar y corrección por altura del objeto."""

import numpy as np


class PlanarPositioner:
    """Convierte coordenadas imagen/plano y aplica corrección de altura."""

    def __init__(self, calibrator):
        """Recibe una instancia calibrada de HomographyCalibrator."""
        self.calibrator = calibrator
        self._camera_center_world = None  # Centro óptico en coords mundo
        self._compute_camera_center()

    def _compute_camera_center(self):
        """Estima el centro de cámara en coordenadas mundo a partir de H."""
        if not self.calibrator.is_calibrated:
            return

        H = self.calibrator.H

        # Descomponer H para obtener la pose de la cámara
        # H = s * K * [r1 r2 t]
        # Sin conocer K, podemos estimar la posición relativa.

        # Normalizar H para que ||h1|| ≈ ||h2|| ≈ 1
        # (asumiendo cámara con píxeles cuadrados y punto principal centrado)
        h1 = H[:, 0]
        h2 = H[:, 1]
        h3 = H[:, 2]

        # Factor de escala
        lambda1 = np.linalg.norm(h1)
        lambda2 = np.linalg.norm(h2)
        lam = (lambda1 + lambda2) / 2.0

        if lam < 1e-10:
            return

        # Vectores de rotación y traslación normalizados
        r1 = h1 / lam
        r2 = h2 / lam
        t = h3 / lam

        # r3 = r1 x r2
        r3 = np.cross(r1, r2)

        # Matriz de rotación (aproximada, no ortogonal perfecta)
        R = np.column_stack([r1, r2, r3])

        # Ortogonalizar R usando SVD
        U, _, Vt = np.linalg.svd(R)
        R = U @ Vt

        # Centro de la cámara en coords mundo: C = -R^T * t
        C = -R.T @ t

        self._camera_center_world = C
        print(f"[Posicionador] Centro de cámara estimado (mundo): "
              f"({C[0]:.1f}, {C[1]:.1f}, {C[2]:.1f}) cm")

    def image_to_plane(self, image_point):
        """Proyecta un punto de imagen al plano de juego (Z=0)."""
        return self.calibrator.image_to_world(image_point)

    def plane_to_image(self, world_point):
        """Proyecta un punto del plano de juego a imagen."""
        return self.calibrator.world_to_image(world_point)

    def correct_height(self, apparent_position, object_height):
        """Corrige la posición planar cuando el centro del objeto está a altura h.

        Usa el factor k = Cz / (Cz - h) sobre la recta entre el centro de cámara
        y la posición aparente obtenida con homografía.
        """
        if apparent_position is None:
            return None

        if abs(object_height) < 1e-6:
            return apparent_position  # No hay corrección necesaria

        if self._camera_center_world is None:
            print("[Posicionador] Advertencia: centro de cámara no estimado. "
                  "No se puede corregir la altura.")
            return apparent_position

        Cx = self._camera_center_world[0]
        Cy = self._camera_center_world[1]
        Cz = self._camera_center_world[2]

        if abs(Cz - object_height) < 1e-6:
            print("[Posicionador] Advertencia: la cámara está a la misma "
                  "altura que el objeto.")
            return apparent_position

        # Factor de corrección
        k = Cz / (Cz - object_height)

        X_corrected = Cx + k * (apparent_position[0] - Cx)
        Y_corrected = Cy + k * (apparent_position[1] - Cy)

        return np.array([X_corrected, Y_corrected])

    def get_ball_world_position(self, image_point, ball_radius_cm=None):
        """Devuelve la posición de la pelota en mundo, con corrección opcional."""
        # Paso 1: Obtener posición aparente sobre el plano
        apparent_pos = self.image_to_plane(image_point)
        if apparent_pos is None:
            return None

        # Paso 2: Corregir altura si se conoce el radio
        if ball_radius_cm is not None and ball_radius_cm > 0:
            return self.correct_height(apparent_pos, ball_radius_cm)

        return apparent_pos

    def set_camera_center(self, camera_center):
        """
        Establece manualmente el centro de la cámara en coordenadas mundo.
        Útil si se conoce la posición exacta de la cámara.

        Args:
            camera_center: [Cx, Cy, Cz] en cm
        """
        self._camera_center_world = np.array(camera_center, dtype=np.float64)

    @staticmethod
    def explain_height_correction():
        """Imprime una explicación detallada de la corrección de altura."""
        explanation = """
        ╔══════════════════════════════════════════════════════════════╗
        ║          CORRECCIÓN DE ALTURA (Apartado 1.4)                 ║
        ╠══════════════════════════════════════════════════════════════╣
        ║                                                              ║
        ║  Problema:                                                   ║
        ║  La homografía H mapea puntos del PLANO (Z=0) a la imagen.   ║
        ║  Si el centro del objeto está a altura h > 0, la posición    ║
        ║  obtenida al aplicar H^(-1) a su proyección en imagen es     ║
        ║  INCORRECTA (desplazada respecto a la posición real).        ║
        ║                                                              ║
        ║            C (cámara)                                        ║
        ║           /|                                                 ║
        ║          / |                                                 ║
        ║         /  |                                                 ║
        ║        /   |                                                 ║
        ║       / h  ● P_real (X,Y,h)                                  ║
        ║      /     |                                                 ║
        ║     /      |                                                 ║
        ║    ●───────●──────── Plano (Z=0)                             ║
        ║  P_aparente  P_proyección_vertical                           ║
        ║                                                              ║
        ║  Solución:                                                   ║
        ║  Conociendo la posición de la cámara C=(Cx,Cy,Cz) y la       ║
        ║  altura h del centro del objeto:                             ║
        ║                                                              ║
        ║    k = Cz / (Cz - h)                                         ║
        ║    X_real = Cx + k × (X_aparente - Cx)                       ║
        ║    Y_real = Cy + k × (Y_aparente - Cy)                       ║
        ║                                                              ║
        ║  Para una pelota de radio r, h = r (centro a altura r).      ║
        ╚══════════════════════════════════════════════════════════════╝
        """
        print(explanation)
