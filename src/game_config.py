"""
config.py - Parámetros de configuración global del proyecto.
Todos los valores están en centímetros (cm) salvo que se indique lo contrario.
"""
import numpy as np

# ============================================================
# CÁMARA
# ============================================================
CAMERA_HTTP_URL = 'http://10.135.115.245/stream'
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FPS = 30

# ============================================================
# ÁREA DE JUEGO (plano de trabajo real, en cm)
# ============================================================
PLAY_AREA_WIDTH = 60.0    # ancho del rectángulo de juego
PLAY_AREA_HEIGHT = 40.0   # alto del rectángulo de juego

# ============================================================
# DETECCIÓN DE PELOTA (rango HSV para ping-pong blanca)
# ============================================================


# Media aproximada: H=24, S=129, V=195
BALL_HSV_LOWER = np.array([20, 90, 165])
BALL_HSV_UPPER = np.array([40, 125, 225])

BALL_MIN_RADIUS_PX = 4       # radio mínimo en píxeles
BALL_MAX_RADIUS_PX = 120     # radio máximo en píxeles
BALL_REAL_RADIUS_CM = 2.0    # radio real de la pelota en cm
BALL_REAL_DIAMETER_CM = BALL_REAL_RADIUS_CM * 2

# ============================================================
# SEGUIMIENTO
# ============================================================
# margen de búsqueda en px alrededor de última posición
TRACKING_SEARCH_MARGIN = 80
TRACKING_METHOD = "color"       # "color" | "csrt" | "kcf" | "mosse" 

# ============================================================
# JUEGO DE MINI-GOLF
# ============================================================
HOLE_RADIUS_CM = 3.5             # radio del hoyo virtual (cm)
HOLE_IN_TOLERANCE_CM = 4.0       # distancia para considerar "embocada"
# Si la pelota desaparece visualmente cerca del hoyo, confirmar embocada
# tras N frames consecutivos no detectada.
HOLE_LOST_FRAMES_CONFIRM = 6
HOLE_LOST_MAX_DIST_CM = 5.0
# umbral de movimiento para considerar la pelota parada
BALL_STOPPED_THRESHOLD_CM = 0.8
BALL_STOPPED_FRAMES = 15         # frames consecutivos parada para confirmar

# Posiciones de los hoyos virtuales (cm) - varios niveles
HOLE_POSITIONS = [
    np.array([45.0, 20.0]),   # Nivel 1
    np.array([15.0, 10.0]),   # Nivel 2
    np.array([50.0, 35.0]),   # Nivel 3
    np.array([10.0, 30.0]),   # Nivel 4
    np.array([30.0, 5.0]),    # Nivel 5
]

# Obstáculos virtuales: lista de (centro_x, centro_y, radio) en cm
OBSTACLES = [
    # (30.0, 20.0, 4.0),   # Obstáculo central (descomentar para usarlos)
]

MAX_PUTTS_PER_HOLE = 10

# ============================================================
# ESCENA VIRTUAL (Open3D usa metros, conversión cm → m)
# ============================================================
SCENE_SCALE = 0.01  # 1 cm = 0.01 m

# Colores RGB normalizados [0,1]
COLOR_BALL = [1.0, 0.5, 0.0]       # Naranja
COLOR_BALL_GHOST = [1.0, 0.8, 0.4]  # Naranja claro (fantasma)
COLOR_HOLE = [0.15, 0.15, 0.15]    # Gris oscuro
COLOR_TABLE = [0.15, 0.55, 0.15]   # Verde (green de golf)
COLOR_TABLE_BORDER = [0.4, 0.25, 0.1]  # Marrón (borde mesa)
COLOR_TRAIL = [1.0, 1.0, 0.0]      # Amarillo (trayectoria)
COLOR_OBSTACLE = [0.6, 0.1, 0.1]   # Rojo oscuro
COLOR_FLAG = [1.0, 0.0, 0.0]       # Rojo (bandera)
COLOR_FLAG_POLE = [0.9, 0.9, 0.9]  # Gris claro (palo)

# ============================================================
# AR (Realidad Aumentada)
# ============================================================
AR_HOLE_COLOR_BGR = (50, 50, 50)
AR_HOLE_BORDER_COLOR_BGR = (0, 0, 0)
AR_FLAG_COLOR_BGR = (0, 0, 255)          # Rojo (BGR)
AR_OBSTACLE_COLOR_BGR = (0, 0, 180)      # Rojo oscuro (BGR)

# ============================================================
# DEMO (cámara simulada)
# ============================================================
DEMO_BG_COLOR_BGR = (40, 140, 40)        # Verde (simula mesa)
DEMO_BALL_COLOR_BGR = (0, 130, 255)      # Naranja (BGR)
DEMO_BALL_SPEED = 5                       # px/frame velocidad teclado
AR_TRAIL_COLOR_BGR = (0, 255, 255)
AR_TEXT_COLOR_BGR = (255, 255, 255)
AR_OBSTACLE_COLOR_BGR = (0, 0, 180)
AR_FLAG_COLOR_BGR = (0, 0, 255)

# ============================================================
# ROBOT / CUBO (modo sin ArUco)
# ============================================================
# Rango HSV de respaldo para detectar la cesta.
ROBOT_CUBE_HSV_LOWER = np.array([110, 50, 160])
ROBOT_CUBE_HSV_UPPER = np.array([125, 120, 255])
ROBOT_CUBE_MIN_AREA_PX = 700
ROBOT_CUBE_MAX_AREA_PX = 120000
ROBOT_HOLE_RADIUS_PX = 24
ROBOT_HOLE_BOX_SCALE = 0.55
ROBOT_HOLE_BOX_COLOR_BGR = (60, 180, 255)

# Ocupacion de cesta (baseline por ROI)
BASKET_OCCUPANCY_DIFF_THRESHOLD = 22
BASKET_OCCUPANCY_DIFF_RATIO = 0.08
BASKET_OCCUPANCY_MAX_SHIFT = 0.35
BASKET_OCCUPANCY_MAX_SIZE_CHANGE = 0.45

# ============================================================
# DEMO / SIMULACIÓN (cámara virtual para pruebas)
# ============================================================
DEMO_MODE = False
DEMO_TABLE_COLOR_BGR = (40, 140, 40)
DEMO_BALL_COLOR_BGR = (0, 255, 0)   # Verde
DEMO_BALL_START = np.array([30.0, 30.0])  # posición inicial en cm


# ============================================================
# PENALTIS DE COCHES
# ============================================================

# Cámaras
CAMERA_LEFT = 0
CAMERA_RIGHT = 2
CAMERA_GESTURE = 0

# Bola verde (HSV) — PyImageSearch
BALL_GREEN_HSV_LOWER = np.array([29, 86, 6])
BALL_GREEN_HSV_UPPER = np.array([64, 255, 255])

GOAL_DEPTH_CM = 12.0

# Juego
MAX_SHOTS = 5
GOAL_COOLDOWN_SEC = 3.0

# Tracking
TRAIL_BUFFER_SIZE = 64
MIN_RADIUS = 10
GAUSSIAN_BLUR_SIZE = (11, 11)
ERODE_ITERATIONS = 2
DILATE_ITERATIONS = 2