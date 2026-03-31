"""Parámetros globales del proyecto.

Todos los valores geométricos están en centímetros, salvo que se indique lo
contrario.
"""

import numpy as np

# -----------------------------------------------------------------------------
# Cámara
# -----------------------------------------------------------------------------
CAMERA_INDEX = 0
# Puede ser índice (int) o URL de stream (str).
CAMERA_SOURCE = CAMERA_INDEX
CAMERA_HTTP_URL = "http://10.135.115.245/stream"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FPS = 30

# -----------------------------------------------------------------------------
# Área de juego (plano real, cm)
# -----------------------------------------------------------------------------
PLAY_AREA_WIDTH = 60.0
PLAY_AREA_HEIGHT = 40.0

# Esquinas del plano en orden: TL, TR, BR, BL.
WORLD_CORNERS = np.array(
    [
        [0.0, 0.0],
        [PLAY_AREA_WIDTH, 0.0],
        [PLAY_AREA_WIDTH, PLAY_AREA_HEIGHT],
        [0.0, PLAY_AREA_HEIGHT],
    ],
    dtype=np.float64,
)

# -----------------------------------------------------------------------------
# Detección de pelota (HSV)
# -----------------------------------------------------------------------------
# Rango (amarillo-verdoso) 
#BALL_HSV_LOWER = np.array([20, 80, 100])
#BALL_HSV_UPPER = np.array([32, 200, 200])

# Rango naranja 
BALL_HSV_LOWER = np.array([5, 120, 120])
BALL_HSV_UPPER = np.array([22, 255, 255])

BALL_MIN_RADIUS_PX = 8
BALL_MAX_RADIUS_PX = 120
BALL_REAL_RADIUS_CM = 2.0
BALL_REAL_DIAMETER_CM = BALL_REAL_RADIUS_CM * 2

# -----------------------------------------------------------------------------
# Seguimiento
# -----------------------------------------------------------------------------
TRACKING_SEARCH_MARGIN = 80
TRACKING_METHOD = "csrt"  # "color" | "csrt" | "kcf" | "mosse"

# -----------------------------------------------------------------------------
# Juego de mini-golf
# -----------------------------------------------------------------------------
HOLE_RADIUS_CM = 3.5
HOLE_IN_TOLERANCE_CM = 4.0
BALL_STOPPED_THRESHOLD_CM = 0.8
BALL_STOPPED_FRAMES = 15

HOLE_POSITIONS = [
    np.array([45.0, 20.0]),
    np.array([15.0, 10.0]),
    np.array([50.0, 35.0]),
    np.array([10.0, 30.0]),
    np.array([30.0, 5.0]),
]

# (centro_x, centro_y, radio) en cm.
OBSTACLES = [
    # (30.0, 20.0, 4.0),
]

MAX_PUTTS_PER_HOLE = 10

# -----------------------------------------------------------------------------
# Escena virtual 3D (Open3D usa metros)
# -----------------------------------------------------------------------------
SCENE_SCALE = 0.01  # 1 cm = 0.01 m

# Colores RGB normalizados [0, 1].
COLOR_BALL = [1.0, 0.5, 0.0]
COLOR_BALL_GHOST = [1.0, 0.8, 0.4]
COLOR_HOLE = [0.15, 0.15, 0.15]
COLOR_TABLE = [0.15, 0.55, 0.15]
COLOR_TABLE_BORDER = [0.4, 0.25, 0.1]
COLOR_TRAIL = [1.0, 1.0, 0.0]
COLOR_OBSTACLE = [0.6, 0.1, 0.1]
COLOR_FLAG = [1.0, 0.0, 0.0]
COLOR_FLAG_POLE = [0.9, 0.9, 0.9]

# -----------------------------------------------------------------------------
# Realidad aumentada (OpenCV usa BGR)
# -----------------------------------------------------------------------------
AR_HOLE_COLOR_BGR = (50, 50, 50)
AR_HOLE_BORDER_COLOR_BGR = (0, 0, 0)
AR_FLAG_COLOR_BGR = (0, 0, 255)
AR_OBSTACLE_COLOR_BGR = (0, 0, 180)
AR_TRAIL_COLOR_BGR = (0, 255, 255)
AR_TEXT_COLOR_BGR = (255, 255, 255)

# -----------------------------------------------------------------------------
# Cámara simulada
# -----------------------------------------------------------------------------
DEMO_MODE = False
DEMO_BG_COLOR_BGR = (40, 140, 40)
DEMO_TABLE_COLOR_BGR = (40, 140, 40)
DEMO_BALL_COLOR_BGR = (0, 130, 255)
DEMO_BALL_SPEED = 5
DEMO_BALL_START = np.array([30.0, 30.0], dtype=np.float64)
