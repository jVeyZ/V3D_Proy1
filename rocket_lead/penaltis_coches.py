"""
Soccer AR Game - PyBullet + Stereo + Gesture (macOS FIX)
Coche cuadrado con ruedas como links fijos (nunca se desincronizan)
"""

import pybullet as p
import pybullet_data
import numpy as np
import cv2
import time
import os
import threading
import queue
import game_config as config

# Garantiza que las rutas relativas (assets/) funcionen
# independientemente del directorio desde el que se lanza el script
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# ─── Queues para comunicación entre hilos ───
ball_queue = queue.Queue(maxsize=5)
stereo_left_queue = queue.Queue(maxsize=2)
stereo_right_queue = queue.Queue(maxsize=2)
gesture_queue = queue.Queue(maxsize=2)

# ─── Estado ArUco compartido (stereo_worker → main thread) ───
_aruco_lock = threading.Lock()
_aruco_ids = {"left": set(), "right": set()}   # IDs detectados actualmente

# ─────────────────────────────────────────────
# CONFIGURACIÓN
# ─────────────────────────────────────────────
FIELD_LENGTH = 12.0
FIELD_WIDTH = 8.0
GOAL_WIDTH = 2.0
GOAL_HEIGHT = 1.2
GOAL_DEPTH = 0.8


# ─────────────────────────────────────────────
# TEXTURA DEL CAMPO
# ─────────────────────────────────────────────
def create_field_texture(path="assets/field_texture.png"):
    os.makedirs("assets", exist_ok=True)
    W, H = 512, 768   # proporción 8:12 del campo
    img = np.zeros((H, W, 3), dtype=np.uint8)

    # Franjas alternas (verde oscuro / verde claro) en dirección larga
    stripe_h = H // 12
    for i in range(12):
        c = (30, 120, 30) if i % 2 == 0 else (45, 160, 45)
        img[i*stripe_h:(i+1)*stripe_h, :] = c

    white = (255, 255, 255)
    lw = 3   # grosor líneas

    margin_x = int(W * 0.04)
    margin_y = int(H * 0.04)

    # Borde exterior
    cv2.rectangle(img, (margin_x, margin_y), (W-margin_x, H-margin_y), white, lw)

    # Línea central
    cy = H // 2
    cv2.line(img, (margin_x, cy), (W-margin_x, cy), white, lw)

    # Círculo central  (radio ~1.5m → ~12% del ancho)
    cr = int(W * 0.12)
    cv2.circle(img, (W//2, cy), cr, white, lw)
    cv2.circle(img, (W//2, cy), 4, white, -1)

    # Áreas de penalti (2m alto = H/6, 4m ancho = W/2)
    pa_h = int(H * 0.14)
    pa_w = int(W * 0.50)
    px0 = W//2 - pa_w//2
    # Arriba
    cv2.rectangle(img, (px0, margin_y), (px0+pa_w, margin_y+pa_h), white, lw)
    # Abajo
    cv2.rectangle(img, (px0, H-margin_y-pa_h), (px0+pa_w, H-margin_y), white, lw)

    # Área pequeña (portería)
    ga_h = int(H * 0.055)
    ga_w = int(W * 0.25)
    gx0 = W//2 - ga_w//2
    cv2.rectangle(img, (gx0, margin_y), (gx0+ga_w, margin_y+ga_h), white, lw)
    cv2.rectangle(img, (gx0, H-margin_y-ga_h), (gx0+ga_w, H-margin_y), white, lw)

    cv2.imwrite(path, img)
    return path


# ─────────────────────────────────────────────
# INICIALIZACIÓN PYBULLET
# ─────────────────────────────────────────────
def init_simulation():
    physicsClient = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)          # oculta ejes y paneles
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.resetDebugVisualizerCamera(15, 60, -30, [0, 0, 0])
    
    # Suelo
    field_tex = create_field_texture()
    plane_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[FIELD_WIDTH/2, FIELD_LENGTH/2, 0.01])
    plane_visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[FIELD_WIDTH/2, FIELD_LENGTH/2, 0.01],
        rgbaColor=[1, 1, 1, 1],
        specularColor=[0.1, 0.1, 0.1]
    )
    texture_id = p.loadTexture(field_tex)
    ground_id = p.createMultiBody(0, plane_shape, plane_visual, basePosition=[0, 0, -0.01])
    p.changeVisualShape(ground_id, -1, textureUniqueId=texture_id)
    
    create_field_boundaries()
    goal_ids = create_simple_goals()
    car_id = create_car()
    p.changeVisualShape(car_id, -1, rgbaColor=[0.9, 0.2, 0.2, 1])
    return physicsClient, car_id, goal_ids


def create_simple_goals():
    """
    Portería con arco blanco mirando al interior del campo
    y red semitransparente hacia el exterior.

    Eje Y del campo: -FIELD_LENGTH/2 (fondo) … +FIELD_LENGTH/2 (frente)
      - Portería norte (y > 0): arco en y_line, red más allá hacia +Y
      - Portería sur  (y < 0): arco en y_line, red más allá hacia -Y
    """
    post_r    = 0.05          # radio de los postes
    cross_r   = 0.04          # radio del larguero

    configs = [
        ( FIELD_LENGTH / 2 - 0.05,  +1),   # portería norte, red va a +Y
        (-FIELD_LENGTH / 2 + 0.05,  -1),   # portería sur,   red va a -Y
    ]

    post_col = p.createCollisionShape(p.GEOM_CYLINDER,
                                       radius=post_r, height=GOAL_HEIGHT)
    post_vis = p.createVisualShape(p.GEOM_CYLINDER,
                                    radius=post_r, length=GOAL_HEIGHT,
                                    rgbaColor=[1, 1, 1, 1])

    cross_col = p.createCollisionShape(p.GEOM_BOX,
                                        halfExtents=[GOAL_WIDTH / 2 + post_r,
                                                     cross_r, cross_r])
    cross_vis = p.createVisualShape(p.GEOM_BOX,
                                     halfExtents=[GOAL_WIDTH / 2 + post_r,
                                                  cross_r, cross_r],
                                     rgbaColor=[1, 1, 1, 1])

    for y_line, net_dir in configs:
        # Postes izquierdo y derecho
        for x_sign in (-1, 1):
            p.createMultiBody(0, post_col, post_vis,
                              [x_sign * GOAL_WIDTH / 2, y_line, GOAL_HEIGHT / 2])

        # Larguero
        p.createMultiBody(0, cross_col, cross_vis,
                          [0, y_line, GOAL_HEIGHT])

        # Red (cubo semitransparente) hacia el exterior del campo
        net_cy = y_line + net_dir * GOAL_DEPTH / 2
        net_col = p.createCollisionShape(p.GEOM_BOX,
                                          halfExtents=[GOAL_WIDTH / 2,
                                                       GOAL_DEPTH / 2,
                                                       GOAL_HEIGHT / 2])
        net_vis = p.createVisualShape(p.GEOM_BOX,
                                       halfExtents=[GOAL_WIDTH / 2,
                                                    GOAL_DEPTH / 2,
                                                    GOAL_HEIGHT / 2],
                                       rgbaColor=[0.85, 0.85, 0.85, 0.25])
        p.createMultiBody(0, net_col, net_vis,
                          [0, net_cy, GOAL_HEIGHT / 2])

    return []


def create_field_boundaries():
    wall_h, wall_thick = 1.0, 0.1
    red = [0.85, 0.1, 0.1, 1.0]
    # Paredes laterales (eje X)
    for x in [-FIELD_WIDTH/2 - wall_thick, FIELD_WIDTH/2 + wall_thick]:
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_thick, FIELD_LENGTH/2, wall_h])
        vis = p.createVisualShape(p.GEOM_BOX,   halfExtents=[wall_thick, FIELD_LENGTH/2, wall_h],
                                   rgbaColor=red)
        p.createMultiBody(0, col, vis, [x, 0, wall_h/2])
    # Paredes de fondo (eje Y)
    for y in [-FIELD_LENGTH/2 - wall_thick, FIELD_LENGTH/2 + wall_thick]:
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[FIELD_WIDTH/2, wall_thick, wall_h])
        vis = p.createVisualShape(p.GEOM_BOX,   halfExtents=[FIELD_WIDTH/2, wall_thick, wall_h],
                                   rgbaColor=red)
        p.createMultiBody(0, col, vis, [0, y, wall_h/2])


def create_car():
    """
    Coche con mesh OBJ (colores del .mtl) + caja de colisión.

    El mesh Blender tiene Y=up, Z=largo. PyBullet usa Z=up, Y=adelante.
    Corrección: R_x(90°) baked en MESH_EXTRA_RPY.

    En CarController.update() el quaternion de orientación = R_z(heading) * R_x(90°)
    para que la rotación visual sea siempre correcta.

    Ajustes si algo no cuadra visualmente:
      MESH_YAW  = np.pi   → morro apunta al revés
      z_center  += 0.05   → coche flota  /  -= 0.05 → se hunde
      SCALE     *= 1.1    → coche demasiado pequeño
    """
    SCALE       = 0.24
    z_center    = 0.19
    MESH_YAW    = np.pi

    start_y = -FIELD_LENGTH / 2 + 1.5

    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.25, 0.56, 0.19])
    vis = p.createVisualShape(
        p.GEOM_MESH,
        fileName="assets/Car.obj",
        meshScale=[SCALE, SCALE, SCALE]
    )

    # Orientación inicial = R_z(0) * R_x(90°)
    base_orn = p.getQuaternionFromEuler([np.pi / 2, 0, MESH_YAW])

    car_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=[0, start_y, z_center],
        baseOrientation=base_orn
    )
    return car_id


def create_tracked_ball():
    radius = 0.27
    col = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=radius,
                               rgbaColor=[0.0, 1.0, 0.0, 0.9])
    ball_id = p.createMultiBody(0, col, vis, basePosition=[0, -5, radius])
    return ball_id


from pynput import keyboard as pynput_kb

# ─────────────────────────────────────────────
# TECLADO FIABLE (pynput — independiente de PyBullet)
# ─────────────────────────────────────────────
_keys_down = set()   # conjunto de teclas actualmente pulsadas

def _on_press(key):
    try:
        _keys_down.add(key.char)
    except AttributeError:
        _keys_down.add(key)

def _on_release(key):
    try:
        _keys_down.discard(key.char)
    except AttributeError:
        _keys_down.discard(key)

_kb_listener = pynput_kb.Listener(on_press=_on_press, on_release=_on_release)
_kb_listener.start()

def key_pressed(c):
    return c in _keys_down
class CarController:
    def __init__(self, car_id):
        self.car_id = car_id
        self.x = 0.0
        self.y = -FIELD_LENGTH / 2 + 1.5
        self.heading = 0.0   # 0 = nariz roja apunta a +Y global

    # Nuestro propio estado de teclas — ignora KEY_IS_DOWN de PyBullet
    # que queda pegado cuando hay lag. Solo KEY_WAS_TRIGGERED / KEY_WAS_RELEASED.
    _key_state = {}

    _key_state    = {}
    _key_time     = {}
    KEY_TIMEOUT   = 0.5

    def update(self, dt):
        dt = min(dt, 0.05)

        speed = 0.0
        if key_pressed('w'):   speed =  2.5
        elif key_pressed('s'): speed = -1.2

        turn = 0.0
        if key_pressed('a'):   turn =  2.0
        elif key_pressed('d'): turn = -2.0

        self.heading += turn * dt
        self.heading %= (2 * np.pi)

        self.x -= speed * np.sin(self.heading) * dt
        self.y += speed * np.cos(self.heading) * dt

        margin = 0.5
        self.x = max(-FIELD_WIDTH/2 + margin, min(FIELD_WIDTH/2 - margin, self.x))
        self.y = max(-FIELD_LENGTH/2 + margin, min(FIELD_LENGTH/2 - margin, self.y))

        z = 0.19
        q_z  = p.getQuaternionFromEuler([0, 0, self.heading + np.pi])
        q_x  = p.getQuaternionFromEuler([np.pi / 2, 0, 0])
        _, orn = p.multiplyTransforms([0,0,0], q_z, [0,0,0], q_x)
        p.resetBasePositionAndOrientation(self.car_id, [self.x, self.y, z], orn)

        # Devolver raw de PyBullet solo para Q/R/C del bucle principal
        raw = p.getKeyboardEvents()
        return raw, [self.x, self.y, z]




# ─────────────────────────────────────────────
# DIBUJAR ARUCOS
# ─────────────────────────────────────────────
def draw_aruco_debug(frame, corners, ids):
    if ids is None or len(ids) == 0:
        return frame
    for (marker_corner, marker_id) in zip(corners, ids.flatten()):
        pts = marker_corner.reshape((4, 2)).astype(int)
        for i in range(4):
            cv2.line(frame, tuple(pts[i]), tuple(pts[(i+1)%4]), (0, 255, 0), 2)
        cX = int((pts[0][0] + pts[2][0]) / 2.0)
        cY = int((pts[0][1] + pts[2][1]) / 2.0)
        cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
        cv2.putText(frame, str(marker_id), (pts[0][0], pts[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return frame


# ─────────────────────────────────────────────
# HILO ESTÉREO (sin cv2.imshow)
# ─────────────────────────────────────────────
def stereo_worker(left_src, right_src, ball_q, left_frame_q, right_frame_q):
    from ball_tracker import StereoGreenTracker
    from stereo import Stereo3DReconstructor
    
    print(f"[CAM] Abriendo cámaras: izq={left_src}, der={right_src}")
    cap_l = cv2.VideoCapture(left_src)
    cap_r = cv2.VideoCapture(right_src)
    
    if not cap_l.isOpened():
        print(f"[CAM] ERROR: No se pudo abrir cámara izquierda ({left_src})")
        return
    if not cap_r.isOpened():
        print(f"[CAM] ERROR: No se pudo abrir cámara derecha ({right_src})")
        return
    
    for cap in [cap_l, cap_r]:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    tracker = StereoGreenTracker()
    reconstructor = Stereo3DReconstructor()
    
    # ArUco detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    try:
        aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    except AttributeError:
        aruco_detector = None
    
    # Calibración
    print("[CAM] Buscando 4 marcadores ArUco...")
    calibrated = False
    attempts = 0
    while not calibrated and attempts < 300:
        ret_l, frame_l = cap_l.read()
        ret_r, frame_r = cap_r.read()
        if ret_l and ret_r:
            calibrated = reconstructor.calibrate_from_aruco(frame_l, frame_r)
        attempts += 1
        if not calibrated:
            time.sleep(0.033)
    
    print(f"[CAM] Stereo: {'CALIBRADO' if calibrated else 'NO CALIBRADO'}")
    
    while True:
        ret_l, frame_l = cap_l.read()
        ret_r, frame_r = cap_r.read()
        if not ret_l or not ret_r:
            time.sleep(0.001)
            continue
        
        # Dibujar ArUcos
        if aruco_detector is not None:
            corners_l, ids_l, _ = aruco_detector.detectMarkers(cv2.cvtColor(frame_l, cv2.COLOR_BGR2GRAY))
            corners_r, ids_r, _ = aruco_detector.detectMarkers(cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY))
            frame_l = draw_aruco_debug(frame_l, corners_l, ids_l)
            frame_r = draw_aruco_debug(frame_r, corners_r, ids_r)
            # Contador de ArUcos visibles
            n_l = len(ids_l) if ids_l is not None else 0
            n_r = len(ids_r) if ids_r is not None else 0
            for frame, n, name in [(frame_l, n_l, "L"), (frame_r, n_r, "R")]:
                color = (0, 255, 0) if n >= 4 else (0, 165, 255) if n > 0 else (0, 0, 255)
                label = f"ArUco {name}: {n}/4"
                fh = frame.shape[0]
                cv2.putText(frame, label, (10, fh - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)
            # Actualizar estado compartido para la ventana de gestos
            with _aruco_lock:
                _aruco_ids["left"]  = set(ids_l.flatten().tolist()) if ids_l is not None else set()
                _aruco_ids["right"] = set(ids_r.flatten().tolist()) if ids_r is not None else set()
        
        # Tracking
        det_l, det_r = tracker.update(frame_l, frame_r)
        pos_3d = None
        if calibrated and det_l is not None and det_r is not None:
            pos_3d = reconstructor.triangulate(det_l[:2], det_r[:2])
            if pos_3d is not None:
                try:
                    ball_q.put_nowait(pos_3d)
                except queue.Full:
                    pass
        
        # Info en frames
        status = "OK" if calibrated else "NO CAL"
        for frame, det, name in [(frame_l, det_l, "L"), (frame_r, det_r, "R")]:
            if det is not None:
                tracker.left.draw_detection(frame, det)
                tracker.left.draw_trail(frame)
                cv2.putText(frame, "BALL", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            else:
                cv2.putText(frame, "NO BALL", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            cv2.putText(frame, f"[{status}] {name}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,200,255), 1)
        
        # Enviar a main thread (NO cv2.imshow aquí)
        try:
            left_frame_q.put_nowait(frame_l)
            right_frame_q.put_nowait(frame_r)
        except queue.Full:
            pass


# ─────────────────────────────────────────────
# HILO GESTOS (sin cv2.imshow)
# ─────────────────────────────────────────────
def gesture_worker(frame_q):
    gesture_available = False
    try:
        from gesture_robot import GestureRobotController
        gesture_available = True
    except Exception as e:
        print(f"[GESTURE] gesture_robot no disponible: {e}")

    if gesture_available:
        controller = GestureRobotController(camera_index=config.CAMERA_GESTURE)
        if controller.start():
            print("[GESTURE] Activo")
            while True:
                frame = controller.get_frame()
                if frame is not None:
                    try:
                        frame_q.put_nowait(frame)
                    except queue.Full:
                        pass
                time.sleep(0.03)
            return  # sólo llega aquí si el bucle termina (no ocurre normalmente)
        else:
            print("[GESTURE] GestureRobotController no pudo iniciarse; fallback a cámara en bruto")

    # ── Fallback: mostrar la cámara de gestos en bruto sin reconocimiento ──
    cap = cv2.VideoCapture(config.CAMERA_GESTURE)
    if not cap.isOpened():
        print(f"[GESTURE] No se pudo abrir cámara {config.CAMERA_GESTURE}")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print("[GESTURE] Mostrando cámara de gestos (sin reconocimiento)")
    while True:
        ret, frame = cap.read()
        if ret:
            reason = "Sin MediaPipe" if not gesture_available else "Tracker no iniciado"
            cv2.putText(frame, f"GESTURE CAM [{reason}]", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            try:
                frame_q.put_nowait(frame)
            except queue.Full:
                pass
        time.sleep(0.033)


# ─────────────────────────────────────────────
# CÁMARA PYBULLET
# ─────────────────────────────────────────────
class CameraController:
    MODES = ['cenital', 'follow']

    def __init__(self):
        self.mode_idx = 0
        self.smooth_pos = np.array([0.0, 0.0, 0.0])

    def get_mode(self):
        return self.MODES[self.mode_idx]

    def next_mode(self):
        self.mode_idx = (self.mode_idx + 1) % len(self.MODES)
        print(f"[CAM] Modo: {self.get_mode()}")

    def update(self, car_pos):
        mode = self.get_mode()
        if mode == 'cenital':
            p.resetDebugVisualizerCamera(18, 90, -89, car_pos)
        elif mode == 'follow':
            target = np.array(car_pos)
            self.smooth_pos += (target - self.smooth_pos) * 0.12
            p.resetDebugVisualizerCamera(6, 180, -30, self.smooth_pos.tolist())


# ─────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────
def main():
    print("=" * 50)
    print("  SOCCER AR GAME - PyBullet + Stereo + Gesture")
    print("=" * 50)
    print("\nControles COCHE:")
    print("  W         : Avanzar (hacia la nariz roja)")
    print("  S         : Retroceder")
    print("  A / D     : Girar sobre el centro")
    print("  R         : Reset")
    print("  C         : Cambiar cámara")
    print("  Q / ESC   : Salir")
    print("\nIniciando...")
    
    physicsClient, car_id, goal_ids = init_simulation()
    car = CarController(car_id)
    cam = CameraController()
    ball_id = create_tracked_ball()
    
    # Lanzar hilos (daemon=True para que mueran al cerrar main)
    t_stereo = threading.Thread(
        target=stereo_worker, 
        args=(config.CAMERA_LEFT, config.CAMERA_RIGHT, 
              ball_queue, stereo_left_queue, stereo_right_queue),
        daemon=True
    )
    t_stereo.start()
    
    t_gesture = threading.Thread(
        target=gesture_worker,
        args=(gesture_queue,),
        daemon=True
    )
    t_gesture.start()
    
    running = True
    last_time = time.time()
    _r_was_down = False
    _c_was_down = False

    # ── Estado de eventos de juego ──
    _msg_text_id   = None   # ID del texto PyBullet en pantalla
    _msg_timer     = 0.0    # segundos restantes del mensaje
    _gol_cooldown  = 0.0    # evitar GOL repetido mientras la bola sigue dentro
    _ball_pb_pos   = [0.0, 0.0, 0.18]   # última posición conocida
    _ball_seen     = False               # True en cuanto llega 1 world_pos real

    def show_message(text, color=(1, 1, 0)):
        nonlocal _msg_text_id, _msg_timer
        if _msg_text_id is not None:
            try:
                p.removeUserDebugItem(_msg_text_id)
            except Exception:
                pass
        _msg_text_id = p.addUserDebugText(
            text, [0, 0, 2.5],
            textColorRGB=color, textSize=3,
            lifeTime=0   # lo gestionamos nosotros
        )
        _msg_timer = 2.5

    while running:
        now = time.time()
        dt = now - last_time
        last_time = now

        # car.update() llama a getKeyboardEvents() internamente y devuelve el dict
        keys, car_pos = car.update(dt)
        cam.update(car_pos)

        if key_pressed('q'):
            running = False
            continue

        if key_pressed('r') and not _r_was_down:
            car.x = 0.0
            car.y = -FIELD_LENGTH / 2 + 1.5
            car.heading = 0.0
            p.resetBasePositionAndOrientation(car_id, [car.x, car.y, 0.19], [0, 0, 0, 1])
            print("[GAME] Reset")
        _r_was_down = key_pressed('r')

        if key_pressed('c') and not _c_was_down:
            cam.next_mode()
        _c_was_down = key_pressed('c')

        # --- Update bola ---
        world_pos = None
        try:
            world_pos = ball_queue.get_nowait()
        except queue.Empty:
            pass

        if world_pos is not None:
            _ball_seen = True
            # world_pos en cm: x ∈ [0, PLAY_AREA_WIDTH], y ∈ [0, PLAY_AREA_HEIGHT]
            x_pb = (world_pos[0] / config.PLAY_AREA_WIDTH  - 0.5) * FIELD_WIDTH
            y_pb = (world_pos[1] / config.PLAY_AREA_HEIGHT - 0.5) * FIELD_LENGTH
            _ball_pb_pos = [x_pb, y_pb, 0.18]
            p.resetBasePositionAndOrientation(
                ball_id, _ball_pb_pos, [0, 0, 0, 1]
            )

        p.stepSimulation()

        # ── Gestión timer del mensaje ──
        _msg_timer -= dt
        if _msg_timer <= 0 and _msg_text_id is not None:
            try:
                p.removeUserDebugItem(_msg_text_id)
            except Exception:
                pass
            _msg_text_id = None
        _gol_cooldown = max(0.0, _gol_cooldown - dt)

        # ── Detección GOL ──
        bx, by, bz = _ball_pb_pos
        in_goal_x = abs(bx) < GOAL_WIDTH / 2
        in_goal_z = bz < GOAL_HEIGHT + 0.1
        if _gol_cooldown == 0.0 and in_goal_x and in_goal_z:
            if by > FIELD_LENGTH / 2 - GOAL_DEPTH:
                show_message("  ¡¡ GOL !!", color=(1, 0.9, 0))
                _gol_cooldown = 3.0
            elif by < -FIELD_LENGTH / 2 + GOAL_DEPTH:
                show_message("  ¡¡ GOL !!", color=(1, 0.9, 0))
                _gol_cooldown = 3.0

        # ── Detección Bola interceptada ──
        # Solo cuando la bola ha sido vista al menos una vez por las cámaras
        if _ball_seen:
            car_pos_arr = np.array([car.x, car.y, 0.19])
            ball_pos_arr = np.array(_ball_pb_pos)
            dist = float(np.linalg.norm(car_pos_arr - ball_pos_arr))
            if dist < 0.65 and _msg_timer <= 0:
                show_message("Bola interceptada", color=(0.2, 0.8, 1))
        
        # ═══════════════════════════════════════════════════════════════
        # MOSTRAR VENTANAS EN MAIN THREAD (macOS lo exige)
        # ═══════════════════════════════════════════════════════════════
        try:
            fl = stereo_left_queue.get_nowait()
            cv2.imshow("Stereo Left", fl)
        except queue.Empty:
            pass
        
        try:
            fr = stereo_right_queue.get_nowait()
            cv2.imshow("Stereo Right", fr)
        except queue.Empty:
            pass
        
        try:
            fg = gesture_queue.get_nowait()
            if fg is not None:
                # ── Panel ArUco (IDs 0-3 esperados) ──────────────────────
                with _aruco_lock:
                    detected = _aruco_ids["left"] | _aruco_ids["right"]
                    in_left  = _aruco_ids["left"].copy()
                    in_right = _aruco_ids["right"].copy()
                panel_x, panel_y = fg.shape[1] - 160, 10
                cv2.rectangle(fg, (panel_x - 6, panel_y - 6),
                              (fg.shape[1] - 4, panel_y + 88), (30, 30, 30), -1)
                cv2.putText(fg, "ArUco (0-3)", (panel_x, panel_y + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                for aid in range(4):
                    ok_l = aid in in_left
                    ok_r = aid in in_right
                    color = (0, 230, 0) if (ok_l or ok_r) else (0, 0, 220)
                    where = ("L+R" if ok_l and ok_r else
                             "L"   if ok_l else
                             "R"   if ok_r else "✗")
                    row = panel_y + 30 + aid * 18
                    cv2.putText(fg, f"#{aid}: {where}", (panel_x, row),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)
                # ─────────────────────────────────────────────────────────
                cv2.imshow("Gesture Robot", fg)
        except queue.Empty:
            pass
        
        # Necesario en macOS para que OpenCV procese eventos de ventana
        cv2.waitKey(1)
        
        time.sleep(max(0, 0.016 - dt))
    
    p.disconnect()
    cv2.destroyAllWindows()
    print("\n[GAME] Cerrando...")


if __name__ == "__main__":
    main()