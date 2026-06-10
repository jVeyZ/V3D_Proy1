"""
Configuración global de Balloon Catch 3D.

Unidades:
- Las dimensiones del volumen de juego están en centímetros.
- Los parámetros de imagen están en píxeles.
"""

import numpy as np

# ============================================================
# VOLUMEN DE JUEGO REAL / MAPEO AL MUNDO VIRTUAL
# ============================================================
PLAY_AREA_WIDTH = 60.0
PLAY_AREA_HEIGHT = 40.0

# ============================================================
# CÁMARAS
# ============================================================
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FPS = 30
CAMERA_LEFT = 0
CAMERA_RIGHT = 2
CAMERA_GESTURE = 0

# ============================================================
# DETECCIÓN DEL GLOBO VERDE
# ============================================================
BALLOON_GREEN_HSV_LOWER = np.array([29, 86, 6])
BALLOON_GREEN_HSV_UPPER = np.array([64, 255, 255])
BALLOON_MIN_RADIUS_PX = 10

# ============================================================
# VALIDACIÓN ESTÉREO
# ============================================================
# Error epipolar máximo aceptado para aceptar una correspondencia izquierda/derecha.
EPIPOLAR_MAX_ERROR_PX = 3.0
# Si la calibración reporta un RMS mayor que este valor, se muestra warning.
STEREO_RMS_WARNING_PX = 2.0

# ============================================================
# TRACKING DEL GLOBO
# ============================================================
TRAIL_BUFFER_SIZE = 64
GAUSSIAN_BLUR_SIZE = (11, 11)
ERODE_ITERATIONS = 2
DILATE_ITERATIONS = 2
