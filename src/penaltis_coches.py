"""
Soccer AR Game - PyBullet + Stereo Vision
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
import src.game_config as config

os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ball_queue = queue.Queue(maxsize=5)
stereo_left_queue = queue.Queue(maxsize=2)
stereo_right_queue = queue.Queue(maxsize=2)
gesture_queue = queue.Queue(maxsize=2)

_hsv_lock = threading.Lock()
_hsv_updated = False
_ball_lock = threading.Lock()
_ball_latest = None
_ball_latest_ts = 0.0

_car_lock = threading.Lock()
_car_pose = None
_car_wire_verts = None
_car_wire_edges = None


# ─────────────────────────────────────────────
# CONFIGURACIÓN
# ─────────────────────────────────────────────
FIELD_LENGTH = 12.0
FIELD_WIDTH = 8.0
GOAL_WIDTH = 2.0
GOAL_HEIGHT = 1.2
GOAL_DEPTH = 0.8

CAR_SCALE = 1.5
CAR_COL_HALF = [0.25 * CAR_SCALE, 0.56 * CAR_SCALE, 0.19 * CAR_SCALE]
CAR_Z_CENTER = 0.19 * CAR_SCALE
CAR_MESH_SCALE = 0.24 * CAR_SCALE
CAR_TOP_Z = CAR_Z_CENTER + CAR_COL_HALF[2]

BALL_RADIUS = 0.40
BALL_COLOR = [0.0, 0.9, 0.0, 0.6]
WORLD_TO_PB = FIELD_WIDTH / config.PLAY_AREA_WIDTH
Z_SCALE = 0.05
HSV_MARGIN = np.array([12, 50, 50], dtype=np.int32)
BALL_SMOOTH_ALPHA = 0.35
BALL_MAX_JUMP = 1.2
Z_DEADZONE_CM = 1.5
Z_MAX_STEP_CM = 3.0
HOLD_LAST_SEC = 1.5
XY_MARGIN_CM = 5.0
Z_MAX_HEIGHT_CM = 60.0
Z_HEIGHT_GAIN = 2.0
Z_ABS_MAX_CM = 150.0
GROUND_LOCK_CM = 2.0
GROUND_ADAPT_ALPHA = 0.15


# ─────────────────────────────────────────────
# TEXTURA DEL CAMPO
# ─────────────────────────────────────────────
def create_field_texture(path="assets/field_texture.png"):
    os.makedirs("assets", exist_ok=True)
    W, H = 512, 768
    img = np.zeros((H, W, 3), dtype=np.uint8)

    stripe_h = H // 12
    for i in range(12):
        c = (30, 120, 30) if i % 2 == 0 else (45, 160, 45)
        img[i*stripe_h:(i+1)*stripe_h, :] = c

    white = (255, 255, 255)
    lw = 3

    margin_x = int(W * 0.04)
    margin_y = int(H * 0.04)

    cv2.rectangle(img, (margin_x, margin_y), (W-margin_x, H-margin_y), white, lw)
    cy = H // 2
    cv2.line(img, (margin_x, cy), (W-margin_x, cy), white, lw)
    cr = int(W * 0.12)
    cv2.circle(img, (W//2, cy), cr, white, lw)
    cv2.circle(img, (W//2, cy), 4, white, -1)

    pa_h = int(H * 0.14)
    pa_w = int(W * 0.50)
    px0 = W//2 - pa_w//2
    cv2.rectangle(img, (px0, margin_y), (px0+pa_w, margin_y+pa_h), white, lw)
    cv2.rectangle(img, (px0, H-margin_y-pa_h), (px0+pa_w, H-margin_y), white, lw)

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
    use_gui = os.environ.get("PYBULLET_DIRECT") not in ("1", "true", "TRUE")
    physicsClient = p.connect(p.GUI if use_gui else p.DIRECT)
    p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.resetDebugVisualizerCamera(15, 60, -30, [0, 0, 0])
    
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
    post_r = 0.05
    cross_r = 0.04

    configs = [
        (FIELD_LENGTH / 2 - 0.05, +1),
        (-FIELD_LENGTH / 2 + 0.05, -1),
    ]

    post_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=post_r, height=GOAL_HEIGHT)
    post_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=post_r, length=GOAL_HEIGHT, rgbaColor=[1, 1, 1, 1])

    cross_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[GOAL_WIDTH / 2 + post_r, cross_r, cross_r])
    cross_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[GOAL_WIDTH / 2 + post_r, cross_r, cross_r], rgbaColor=[1, 1, 1, 1])

    for y_line, net_dir in configs:
        for x_sign in (-1, 1):
            p.createMultiBody(0, post_col, post_vis, [x_sign * GOAL_WIDTH / 2, y_line, GOAL_HEIGHT / 2])
        p.createMultiBody(0, cross_col, cross_vis, [0, y_line, GOAL_HEIGHT])

        net_cy = y_line + net_dir * GOAL_DEPTH / 2
        net_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[GOAL_WIDTH / 2, GOAL_DEPTH / 2, GOAL_HEIGHT / 2])
        net_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[GOAL_WIDTH / 2, GOAL_DEPTH / 2, GOAL_HEIGHT / 2], rgbaColor=[0.85, 0.85, 0.85, 0.25])
        p.createMultiBody(0, net_col, net_vis, [0, net_cy, GOAL_HEIGHT / 2])
    return []


def create_field_boundaries():
    wall_h, wall_thick = 1.0, 0.1
    red = [0.85, 0.1, 0.1, 1.0]
    for x in [-FIELD_WIDTH/2 - wall_thick, FIELD_WIDTH/2 + wall_thick]:
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_thick, FIELD_LENGTH/2, wall_h])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_thick, FIELD_LENGTH/2, wall_h], rgbaColor=red)
        p.createMultiBody(0, col, vis, [x, 0, wall_h/2])
    for y in [-FIELD_LENGTH/2 - wall_thick, FIELD_LENGTH/2 + wall_thick]:
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[FIELD_WIDTH/2, wall_thick, wall_h])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[FIELD_WIDTH/2, wall_thick, wall_h], rgbaColor=red)
        p.createMultiBody(0, col, vis, [0, y, wall_h/2])


def create_car():
    SCALE = 0.24
    z_center = 0.19
    MESH_YAW = np.pi
    start_y = -FIELD_LENGTH / 2 + 1.5

    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=CAR_COL_HALF)
    vis = p.createVisualShape(
        p.GEOM_MESH,
        fileName="assets/Car.obj",
        meshScale=[CAR_MESH_SCALE, CAR_MESH_SCALE, CAR_MESH_SCALE]
    )

    base_orn = p.getQuaternionFromEuler([np.pi / 2, 0, MESH_YAW])
    car_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=[0, start_y, CAR_Z_CENTER],
        baseOrientation=base_orn
    )
    return car_id


def create_tracked_ball():
    radius = BALL_RADIUS
    col = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    vis = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=BALL_COLOR,
    )
    ball_id = p.createMultiBody(0, col, vis, basePosition=[0, -5, radius])
    return ball_id


from pynput import keyboard as pynput_kb

# ─────────────────────────────────────────────
# TECLADO FIABLE (pynput)
# ─────────────────────────────────────────────
_keys_down = set()

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
        self.heading = 0.0

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

        z = CAR_Z_CENTER
        q_z  = p.getQuaternionFromEuler([0, 0, self.heading + np.pi])
        q_x  = p.getQuaternionFromEuler([np.pi / 2, 0, 0])
        _, orn = p.multiplyTransforms([0,0,0], q_z, [0,0,0], q_x)
        p.resetBasePositionAndOrientation(self.car_id, [self.x, self.y, z], orn)

        return {}, [self.x, self.y, z]


# ─────────────────────────────────────────────
# HILO ESTÉREO
# ─────────────────────────────────────────────
def stereo_worker(left_src, right_src, ball_q, left_frame_q, right_frame_q):
    from src.ball_tracker import StereoGreenTracker
    from src.stereo import Stereo3DReconstructor
    
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
    calib_path = os.path.join(os.getcwd(), "calibration", "stereo_charuco.npz")
    if os.path.exists(calib_path):
        if reconstructor.load_calibration(calib_path):
            print(f"[CAL] Cargada calibración estéreo desde {calib_path}")
        else:
            print(f"[CAL] No se pudo cargar calibración desde {calib_path}")
    
    calibrated = reconstructor.is_calibrated
    if calibrated:
        print("[CAM] Iniciando — calibración estéreo cargada (sin ArUco)")
        print(f"[CAM]   K_left=\n{reconstructor.K_left}")
        print(f"[CAM]   K_right=\n{reconstructor.K_right}")
    else:
        print("[CAM] Iniciando — SIN calibración estéreo")
    print("[CAM] Para calibrar color de la bola: pulsa 'P' y selecciona un rectángulo sobre ella")
    print(f"[CAM] HSV bola verde actual: {config.BALL_GREEN_HSV_LOWER.tolist()} → {config.BALL_GREEN_HSV_UPPER.tolist()}")
    
    frame_count = 0

    while True:
        ret_l, frame_l = cap_l.read()
        ret_r, frame_r = cap_r.read()
        if not ret_l or not ret_r:
            time.sleep(0.001)
            continue
        
        # Tracking de bola verde
        with _hsv_lock:
            global _hsv_updated
            if _hsv_updated:
                tracker.reset()
                _hsv_updated = False
        det_l, det_r = tracker.update(frame_l, frame_r)
        
        if frame_count % 30 == 0:
            print(f"[DET] L={'✓' if det_l is not None else '✗'}  "
                  f"R={'✓' if det_r is not None else '✗'}  "
                  f"{'3D' if calibrated else 'NO-CAL'}")
        
        # Triangulación estéreo pura
        pos_3d = None
        mode_label = None
        tri = None

        if calibrated and det_l is not None and det_r is not None:
            tri = reconstructor.triangulate(det_l[:2], det_r[:2])
            if tri is not None and np.isfinite(tri).all():
                pos_3d = tri.copy()
                mode_label = "3D"

        if pos_3d is not None:
            with _ball_lock:
                global _ball_latest, _ball_latest_ts
                _ball_latest = pos_3d
                _ball_latest_ts = time.time()
            if frame_count % 15 == 0:
                print(f"[POS] {mode_label}: "
                      f"X={pos_3d[0]:6.1f}  Y={pos_3d[1]:6.1f}  Z={pos_3d[2]:6.1f}")
            try:
                ball_q.put_nowait(pos_3d)
            except queue.Full:
                try:
                    ball_q.get_nowait()
                except queue.Empty:
                    pass
                try:
                    ball_q.put_nowait(pos_3d)
                except queue.Full:
                    pass
        
        # Info en frames
        cal_status = "3D" if calibrated else "NO-CAL"
        for frame, det, name, tracker_side in [
            (frame_l, det_l, "L", tracker.left),
            (frame_r, det_r, "R", tracker.right)
        ]:
            if det is not None:
                tracker_side.draw_detection(frame, det)
                tracker_side.draw_trail(frame)
                cv2.putText(frame, "BALL", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            else:
                cv2.putText(frame, "NO BALL", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            cv2.putText(frame, f"[{cal_status}] {name}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,200,255), 1)
            if mode_label is not None:
                cv2.putText(frame, mode_label, (10, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,0), 1)
            if pos_3d is not None:
                cv2.putText(frame, f"X={pos_3d[0]:.1f} Y={pos_3d[1]:.1f} Z={pos_3d[2]:.1f}",
                            (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,0), 2)
        
        try:
            left_frame_q.put_nowait(frame_l)
            right_frame_q.put_nowait(frame_r)
        except queue.Full:
            pass

        frame_count += 1


# ─────────────────────────────────────────────
# HILO GESTOS
# ─────────────────────────────────────────────
def gesture_worker(frame_q):
    gesture_available = False
    try:
        from src.gesture_robot import GestureRobotController
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
            return
        else:
            print("[GESTURE] GestureRobotController no pudo iniciarse; fallback a cámara en bruto")

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
            cv2.putText(frame, "GESTURE CAM [Sin MediaPipe]", (10, 30),
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


class ROISelector:
    def __init__(self):
        self.enabled = False
        self.dragging = False
        self.start = None
        self.end = None
        self.ready = False

    def enable(self):
        self.enabled = True
        self.dragging = False
        self.start = None
        self.end = None
        self.ready = False

    def disable(self):
        self.enabled = False
        self.dragging = False
        self.start = None
        self.end = None
        self.ready = False

    def on_mouse(self, event, x, y, _flags, _param):
        if not self.enabled:
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.start = (x, y)
            self.end = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            self.end = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.dragging:
            self.end = (x, y)
            self.dragging = False
            self.ready = True

    def get_roi(self):
        if self.start is None or self.end is None:
            return None
        x0 = min(self.start[0], self.end[0])
        y0 = min(self.start[1], self.end[1])
        x1 = max(self.start[0], self.end[0])
        y1 = max(self.start[1], self.end[1])
        w = x1 - x0
        h = y1 - y0
        if w <= 5 or h <= 5:
            return None
        return (x0, y0, w, h)

    def draw(self, frame):
        if not self.enabled or self.start is None or self.end is None:
            return
        x0 = min(self.start[0], self.end[0])
        y0 = min(self.start[1], self.end[1])
        x1 = max(self.start[0], self.end[0])
        y1 = max(self.start[1], self.end[1])
        cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 255), 2)


def _set_hsv_from_roi(frame_bgr, roi):
    x, y, w, h = roi
    h_img, w_img = frame_bgr.shape[:2]
    x1 = max(0, min(w_img, x + w))
    y1 = max(0, min(h_img, y + h))
    x0 = max(0, min(w_img - 1, x))
    y0 = max(0, min(h_img - 1, y))
    if x1 <= x0 or y1 <= y0:
        return False
    crop = frame_bgr[y0:y1, x0:x1]
    if crop.size == 0:
        return False
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    center = np.median(hsv.reshape(-1, 3), axis=0).astype(np.int32)
    lower = np.clip(center - HSV_MARGIN, 0, 255).astype(np.uint8)
    upper = np.clip(center + HSV_MARGIN, 0, 255).astype(np.uint8)
    with _hsv_lock:
        global _hsv_updated
        config.BALL_GREEN_HSV_LOWER = lower
        config.BALL_GREEN_HSV_UPPER = upper
        _hsv_updated = True
    return True


# ─────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────
def main():
    print("=" * 50)
    print("  SOCCER AR GAME - PyBullet + Stereo")
    print("=" * 50)
    print("\nControles COCHE:")
    print("  W         : Avanzar (hacia la nariz roja)")
    print("  S         : Retroceder")
    print("  A / D     : Girar sobre el centro")
    print("  R         : Reset")
    print("  C         : Cambiar cámara")
    print("  G         : Calibrar suelo (poner bola en el suelo y pulsar)")
    print("  Q / ESC   : Salir")
    print("\nIniciando...")
    
    physicsClient, car_id, goal_ids = init_simulation()
    car = CarController(car_id)
    cam = CameraController()
    ball_id = create_tracked_ball()
    
    with _car_lock:
        _car_pose = (car.x, car.y, car.heading)
    
    # Lanzar hilos
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
    _p_was_down = False
    _g_was_down = False
    _g_request = False

    roi_selector = ROISelector()
    left_window = "Stereo Left"
    right_window = "Stereo Right"
    left_window_ready = False
    right_window_ready = False
    last_left_frame = None

    _ball_pb_pos   = [0.0, 0.0, BALL_RADIUS]
    _ground_z_cm = None
    _ball_smooth = None
    _ball_smooth_z = None

    while running:
        now = time.time()
        dt = now - last_time
        last_time = now

        keys, car_pos = car.update(dt)
        cam.update(car_pos)
        with _car_lock:
            _car_pose = (car.x, car.y, car.heading)

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

        if key_pressed('g') and not _g_was_down:
            _g_request = True
            print("[BALL] Calibrando suelo: pon la bola en el suelo y pulsa G")
        _g_was_down = key_pressed('g')

        # --- Update bola ---
        world_pos = None
        try:
            while True:
                world_pos = ball_queue.get_nowait()
        except queue.Empty:
            pass

        if world_pos is None:
            with _ball_lock:
                if _ball_latest is not None and (now - _ball_latest_ts) < HOLD_LAST_SEC:
                    world_pos = _ball_latest

        z_bad = False
        if world_pos is not None:
            try:
                x_cm = float(world_pos[0])
                y_cm = float(world_pos[1])
                z_cm = float(world_pos[2])
            except Exception:
                world_pos = None

        if world_pos is not None and not np.isfinite([x_cm, y_cm, z_cm]).all():
            world_pos = None
            z_bad = True

        if world_pos is not None:
            if (x_cm < -XY_MARGIN_CM or x_cm > config.PLAY_AREA_WIDTH + XY_MARGIN_CM or
                y_cm < -XY_MARGIN_CM or y_cm > config.PLAY_AREA_HEIGHT + XY_MARGIN_CM):
                world_pos = None
            elif abs(z_cm) > Z_ABS_MAX_CM:
                z_bad = True

        if world_pos is not None:
            x_cm = min(max(x_cm, 0.0), config.PLAY_AREA_WIDTH)
            y_cm = min(max(y_cm, 0.0), config.PLAY_AREA_HEIGHT)

            if z_bad:
                if _ground_z_cm is not None:
                    z_cm = _ground_z_cm
                else:
                    z_cm = 0.0

            if _g_request:
                _ground_z_cm = z_cm
                _ball_smooth_z = None
                _g_request = False
                print(f"[BALL] Suelo calibrado manual: {_ground_z_cm:.1f} cm")

            if _ground_z_cm is None:
                _ground_z_cm = z_cm
                print(f"[BALL] Suelo inicial automático: {_ground_z_cm:.1f} cm")

            height_cm_raw = abs(z_cm - _ground_z_cm)
            if height_cm_raw < Z_DEADZONE_CM:
                height_cm_raw = 0.0
            if height_cm_raw < GROUND_LOCK_CM:
                _ground_z_cm = (1.0 - GROUND_ADAPT_ALPHA) * _ground_z_cm + GROUND_ADAPT_ALPHA * z_cm
            height_cm = min(height_cm_raw * Z_HEIGHT_GAIN, Z_MAX_HEIGHT_CM)

            x_pb = (x_cm / config.PLAY_AREA_WIDTH  - 0.5) * FIELD_WIDTH
            y_pb = (y_cm / config.PLAY_AREA_HEIGHT - 0.5) * FIELD_LENGTH
            z_pb = BALL_RADIUS + height_cm * Z_SCALE

            if _ball_smooth_z is None:
                _ball_smooth_z = z_pb
            else:
                max_step = Z_MAX_STEP_CM * Z_SCALE
                delta_z = z_pb - _ball_smooth_z
                if abs(delta_z) > max_step:
                    z_pb = _ball_smooth_z + np.sign(delta_z) * max_step
                _ball_smooth_z += (z_pb - _ball_smooth_z) * 0.3
                z_pb = _ball_smooth_z

            raw_pos = np.array([x_pb, y_pb, z_pb], dtype=np.float64)
            if _ball_smooth is None:
                _ball_smooth = raw_pos
            else:
                delta = raw_pos - _ball_smooth
                if np.linalg.norm(delta) > BALL_MAX_JUMP:
                    raw_pos = _ball_smooth + delta * 0.2
                _ball_smooth = _ball_smooth + (raw_pos - _ball_smooth) * BALL_SMOOTH_ALPHA

            _ball_pb_pos = _ball_smooth.tolist()
            _ball_smooth_z = float(_ball_smooth[2])
            p.resetBasePositionAndOrientation(
                ball_id, _ball_pb_pos, [0, 0, 0, 1]
            )

        p.stepSimulation()
        
        # ═══════════════════════════════════════════════════════════════
        # MOSTRAR VENTANAS EN MAIN THREAD (macOS lo exige)
        # ═══════════════════════════════════════════════════════════════
        try:
            fl = stereo_left_queue.get_nowait()
            last_left_frame = fl
            if not left_window_ready:
                cv2.namedWindow(left_window, cv2.WINDOW_NORMAL)
                cv2.setMouseCallback(left_window, roi_selector.on_mouse)
                left_window_ready = True
            if roi_selector.enabled:
                roi_selector.draw(fl)
                cv2.putText(fl, "ROI MODE", (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 255), 2)
            cv2.imshow(left_window, fl)
        except queue.Empty:
            pass
        
        try:
            fr = stereo_right_queue.get_nowait()
            if not right_window_ready:
                cv2.namedWindow(right_window, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(right_window, 480, 360)
                right_window_ready = True
            cv2.imshow(right_window, fr)
        except queue.Empty:
            pass
        
        try:
            fg = gesture_queue.get_nowait()
            if fg is not None:
                cv2.imshow("Gesture Robot", fg)
        except queue.Empty:
            pass
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('p'):
            if last_left_frame is not None:
                roi = cv2.selectROI(left_window, last_left_frame, fromCenter=False, showCrosshair=True)
                if roi is not None and roi[2] > 5 and roi[3] > 5:
                    if _set_hsv_from_roi(last_left_frame, roi):
                        print("[ROI] HSV del globo actualizado")
                    else:
                        print("[ROI] Seleccion invalida")
                else:
                    print("[ROI] Seleccion invalida")
            else:
                print("[ROI] Espera a que llegue un frame en 'Stereo Left'")

        time.sleep(max(0, 0.016 - dt))
    
    p.disconnect()
    cv2.destroyAllWindows()
    print("\n[GAME] Cerrando...")


if __name__ == "__main__":
    main()
