"""
Configuración global de Balloon Catch 3D.

Unidades:
- Coordenadas 3D trianguladas en centímetros (sistema del tablero ChArUco en el suelo).
- 1 cm real = WORLD_SCALE metros en PyBullet.
- Parámetros de imagen en píxeles.
"""

import numpy as np

# ============================================================
# MAPEO MUNDO REAL → PYBULLET
# ============================================================
# 1 cm en el mundo real = WORLD_SCALE metros en el gemelo digital
WORLD_SCALE = 0.05

# ── Remapeo de ejes del mundo ──
# Ajusta para alinear con el suelo real:
#   X = horizontal (izquierda → derecha)
#   Y = profundidad (cámara → fondo)
#   Z = altura desde el suelo
WORLD_SWAP_XY = True
WORLD_FLIP_X = False
WORLD_FLIP_Y = False
WORLD_FLIP_Z = False

# ============================================================
# CÁMARAS
# ============================================================
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FPS = 30
# Cada fuente puede ser un índice local (0, 1, 2, ...) o una URL HTTP/RTSP.
# Ejemplo móvil Android con IP Webcam: CAMERA_RIGHT = "http://192.168.1.50:8080/video"
CAMERA_LEFT = 0
CAMERA_RIGHT = 1
CAMERA_GESTURE = 2

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
