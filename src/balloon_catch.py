"""
Balloon Catch 3D - PyBullet + Stereo Vision + Gesture Control

Juego Proyecto 2:
- Un globo verde real se detecta en dos cámaras estéreo.
- Su centro 3D se triangula íntegramente con estéreo y se mapea al mundo virtual.
- Un robot/cesta digital se controla mediante gestos o teclado.
- Objetivo: situar el robot debajo del globo cuando toque el suelo.
"""

import os
import queue
import threading
import time

import cv2
import numpy as np
import pybullet as p
import pybullet_data
from pynput import keyboard as pynput_kb

import src.game_config as config

os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

balloon_queue = queue.Queue(maxsize=5)
stereo_left_queue = queue.Queue(maxsize=2)
stereo_right_queue = queue.Queue(maxsize=2)
gesture_queue = queue.Queue(maxsize=2)
gesture_command_queue = queue.Queue(maxsize=5)

_hsv_lock = threading.Lock()
_hsv_updated = False
_balloon_lock = threading.Lock()
_balloon_latest = None
_balloon_latest_ts = 0.0
_robot_world_lock = threading.Lock()
_robot_world_pose = np.array([config.PLAY_AREA_WIDTH / 2, 0.0, 0.0], dtype=np.float64)

# ─────────────────────────────────────────────
# CONFIGURACIÓN
# ─────────────────────────────────────────────
FIELD_LENGTH = 12.0
FIELD_WIDTH = 8.0
ROBOT_SCALE = 1.5
ROBOT_HALF_EXTENTS = [0.25 * ROBOT_SCALE, 0.56 * ROBOT_SCALE, 0.19 * ROBOT_SCALE]
ROBOT_Z_CENTER = 0.19 * ROBOT_SCALE
ROBOT_MESH_SCALE = 0.24 * ROBOT_SCALE
BALLOON_RADIUS = 0.40
BALLOON_COLOR = [0.0, 0.95, 0.1, 0.65]
BALLOON_CAUGHT_COLOR = [0.1, 0.8, 1.0, 0.85]
ROBOT_COLOR = [0.9, 0.2, 0.2, 1.0]
CATCH_RADIUS = 0.95
GROUND_TOUCH_HEIGHT_CM = 8.0
CATCH_COOLDOWN_SEC = 1.5
Z_SCALE = 0.05
HSV_MARGIN = np.array([12, 50, 50], dtype=np.int32)
BALLOON_SMOOTH_ALPHA = 0.35
BALLOON_MAX_JUMP = 1.2
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
        img[i * stripe_h : (i + 1) * stripe_h, :] = c

    white = (255, 255, 255)
    lw = 3

    margin_x = int(W * 0.04)
    margin_y = int(H * 0.04)

    cv2.rectangle(img, (margin_x, margin_y), (W - margin_x, H - margin_y), white, lw)
    cy = H // 2
    cv2.line(img, (margin_x, cy), (W - margin_x, cy), white, lw)
    cr = int(W * 0.12)
    cv2.circle(img, (W // 2, cy), cr, white, lw)
    cv2.circle(img, (W // 2, cy), 4, white, -1)

    pa_h = int(H * 0.14)
    pa_w = int(W * 0.50)
    px0 = W // 2 - pa_w // 2
    cv2.rectangle(img, (px0, margin_y), (px0 + pa_w, margin_y + pa_h), white, lw)
    cv2.rectangle(
        img, (px0, H - margin_y - pa_h), (px0 + pa_w, H - margin_y), white, lw
    )

    ga_h = int(H * 0.055)
    ga_w = int(W * 0.25)
    gx0 = W // 2 - ga_w // 2
    cv2.rectangle(img, (gx0, margin_y), (gx0 + ga_w, margin_y + ga_h), white, lw)
    cv2.rectangle(
        img, (gx0, H - margin_y - ga_h), (gx0 + ga_w, H - margin_y), white, lw
    )

    cv2.imwrite(path, img)
    return path


# ─────────────────────────────────────────────
# INICIALIZACIÓN PYBULLET
# ─────────────────────────────────────────────
def init_simulation():
    use_gui = os.environ.get("PYBULLET_DIRECT") not in ("1", "true", "TRUE")
    physics_client = p.connect(p.GUI if use_gui else p.DIRECT)
    p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.resetDebugVisualizerCamera(15, 60, -30, [0, 0, 0])

    field_tex = create_field_texture()
    plane_shape = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[FIELD_WIDTH / 2, FIELD_LENGTH / 2, 0.01]
    )
    plane_visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[FIELD_WIDTH / 2, FIELD_LENGTH / 2, 0.01],
        rgbaColor=[1, 1, 1, 1],
        specularColor=[0.1, 0.1, 0.1],
    )
    texture_id = p.loadTexture(field_tex)
    ground_id = p.createMultiBody(
        0, plane_shape, plane_visual, basePosition=[0, 0, -0.01]
    )
    p.changeVisualShape(ground_id, -1, textureUniqueId=texture_id)

    create_field_boundaries()
    robot_id = create_robot()
    p.changeVisualShape(robot_id, -1, rgbaColor=ROBOT_COLOR)
    return physics_client, robot_id


def create_field_boundaries():
    wall_h, wall_thick = 1.0, 0.1
    red = [0.85, 0.1, 0.1, 1.0]
    for x in [-FIELD_WIDTH / 2 - wall_thick, FIELD_WIDTH / 2 + wall_thick]:
        col = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[wall_thick, FIELD_LENGTH / 2, wall_h]
        )
        vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[wall_thick, FIELD_LENGTH / 2, wall_h],
            rgbaColor=red,
        )
        p.createMultiBody(0, col, vis, [x, 0, wall_h / 2])
    for y in [-FIELD_LENGTH / 2 - wall_thick, FIELD_LENGTH / 2 + wall_thick]:
        col = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[FIELD_WIDTH / 2, wall_thick, wall_h]
        )
        vis = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[FIELD_WIDTH / 2, wall_thick, wall_h], rgbaColor=red
        )
        p.createMultiBody(0, col, vis, [0, y, wall_h / 2])


def create_robot():
    """Crea el robot/cesta digital.

    Si existe assets/Robot.obj se usa como malla visual. Si no, se usa una caja
    PyBullet nativa para que la demo sea autocontenida.
    """
    mesh_path = "assets/Robot.obj"
    start_y = -FIELD_LENGTH / 2 + 1.5

    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=ROBOT_HALF_EXTENTS)
    if os.path.exists(mesh_path):
        vis = p.createVisualShape(
            p.GEOM_MESH,
            fileName=mesh_path,
            meshScale=[ROBOT_MESH_SCALE, ROBOT_MESH_SCALE, ROBOT_MESH_SCALE],
            rgbaColor=ROBOT_COLOR,
        )
        base_orn = p.getQuaternionFromEuler([np.pi / 2, 0, np.pi])
    else:
        vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=ROBOT_HALF_EXTENTS,
            rgbaColor=ROBOT_COLOR,
        )
        base_orn = p.getQuaternionFromEuler([0, 0, 0])

    robot_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=[0, start_y, ROBOT_Z_CENTER],
        baseOrientation=base_orn,
    )
    return robot_id


def create_tracked_balloon():
    radius = BALLOON_RADIUS
    col = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    vis = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=BALLOON_COLOR,
    )
    balloon_id = p.createMultiBody(0, col, vis, basePosition=[0, -5, radius])
    return balloon_id


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


def pybullet_robot_to_world_cm(robot_x, robot_y):
    """Convierte posición PyBullet del robot a coordenadas del mundo de juego en cm."""
    world_x = (robot_x / FIELD_WIDTH + 0.5) * config.PLAY_AREA_WIDTH
    world_y = (robot_y / FIELD_LENGTH + 0.5) * config.PLAY_AREA_HEIGHT
    return np.array([world_x, world_y, 0.0], dtype=np.float64)


def draw_robot_ar_overlay(frame, reconstructor):
    """Proyecta la huella del robot virtual sobre la cámara izquierda."""
    if not reconstructor.has_world_transform:
        return
    with _robot_world_lock:
        center = _robot_world_pose.copy()

    half_x_cm = ROBOT_HALF_EXTENTS[0] / FIELD_WIDTH * config.PLAY_AREA_WIDTH
    half_y_cm = ROBOT_HALF_EXTENTS[1] / FIELD_LENGTH * config.PLAY_AREA_HEIGHT
    corners_world = [
        center + np.array([-half_x_cm, -half_y_cm, 0.0]),
        center + np.array([half_x_cm, -half_y_cm, 0.0]),
        center + np.array([half_x_cm, half_y_cm, 0.0]),
        center + np.array([-half_x_cm, half_y_cm, 0.0]),
    ]
    projected = [reconstructor.project_world_to_left(pt) for pt in corners_world]
    if any(pt is None for pt in projected):
        return
    pts = np.array(projected, dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(frame, [pts], isClosed=True, color=(255, 80, 40), thickness=2)
    center_px = reconstructor.project_world_to_left(center)
    if center_px is not None:
        cv2.circle(frame, center_px, 5, (255, 80, 40), -1)
        cv2.putText(
            frame,
            "ROBOT AR",
            (center_px[0] + 8, center_px[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 80, 40),
            2,
        )


class RobotController:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.x = 0.0
        self.y = -FIELD_LENGTH / 2 + 1.5
        self.heading = 0.0
        self.last_gesture = None

    def update(self, dt, gesture_payload=None):
        dt = min(dt, 0.05)

        speed = 0.0
        turn = 0.0

        # Control manual de respaldo.
        if key_pressed("w"):
            speed = 2.5
        elif key_pressed("s"):
            speed = -1.2
        if key_pressed("a"):
            turn = 2.0
        elif key_pressed("d"):
            turn = -2.0

        # Control diferencial por gestos MediaPipe. Si hay gesto activo, toma prioridad.
        if gesture_payload is not None:
            left_cmd = gesture_payload.get("command_left")
            right_cmd = gesture_payload.get("command_right")
            left_speed = self._wheel_speed(left_cmd)
            right_speed = self._wheel_speed(right_cmd)
            if left_cmd is not None or right_cmd is not None:
                speed = (left_speed + right_speed) * 1.2
                turn = (right_speed - left_speed) * 1.8
                self.last_gesture = (left_cmd, right_cmd)

        self.heading += turn * dt
        self.heading %= 2 * np.pi

        self.x -= speed * np.sin(self.heading) * dt
        self.y += speed * np.cos(self.heading) * dt

        margin = 0.5
        self.x = max(-FIELD_WIDTH / 2 + margin, min(FIELD_WIDTH / 2 - margin, self.x))
        self.y = max(-FIELD_LENGTH / 2 + margin, min(FIELD_LENGTH / 2 - margin, self.y))

        z = ROBOT_Z_CENTER
        q_z = p.getQuaternionFromEuler([0, 0, self.heading + np.pi])
        q_x = p.getQuaternionFromEuler([np.pi / 2, 0, 0])
        _, orn = p.multiplyTransforms([0, 0, 0], q_z, [0, 0, 0], q_x)
        p.resetBasePositionAndOrientation(self.robot_id, [self.x, self.y, z], orn)

        return {}, [self.x, self.y, z]

    @staticmethod
    def _wheel_speed(command):
        if command in ("forward_left", "forward_right"):
            return 1.0
        if command in ("backward_left", "backward_right"):
            return -0.7
        return 0.0


# ─────────────────────────────────────────────
# HILO ESTÉREO
# ─────────────────────────────────────────────
def stereo_worker(left_src, right_src, balloon_q, left_frame_q, right_frame_q):
    from src.balloon_tracker import StereoGreenBalloonTracker
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

    tracker = StereoGreenBalloonTracker()
    reconstructor = Stereo3DReconstructor()
    calib_path = os.path.join(os.getcwd(), "calibration", "stereo_charuco.npz")
    world_transform_path = os.path.join(
        os.getcwd(), "calibration", "world_transform.npz"
    )
    if os.path.exists(calib_path):
        if reconstructor.load_calibration(calib_path):
            print(f"[CAL] Cargada calibración estéreo desde {calib_path}")
            summary = reconstructor.calibration_summary()
            print(
                f"[CAL] RMS stereo={summary['rms_stereo']} px | baseline={summary['baseline_cm']} cm"
            )
            if (
                summary["rms_stereo"] is not None
                and summary["rms_stereo"] > config.STEREO_RMS_WARNING_PX
            ):
                print(
                    "[CAL] WARNING: RMS estéreo alto; recalibra antes de la presentación"
                )
        else:
            print(f"[CAL] No se pudo cargar calibración desde {calib_path}")

    if reconstructor.is_calibrated and os.path.exists(world_transform_path):
        if reconstructor.load_world_transform(world_transform_path):
            print(f"[CAL] Cargada transformación de mundo desde {world_transform_path}")
            if reconstructor.world_rms_cm is not None:
                print(
                    f"[CAL] RMS transformación mundo={reconstructor.world_rms_cm:.3f} cm"
                )

    calibrated = reconstructor.is_calibrated
    if calibrated:
        print("[CAM] Iniciando — calibración estéreo cargada (sin ArUco en runtime)")
        print(f"[CAM]   K_left=\n{reconstructor.K_left}")
        print(f"[CAM]   K_right=\n{reconstructor.K_right}")
        if reconstructor.has_world_transform:
            print("[CAM]   Sistema de mundo: calibration/world_transform.npz")
        else:
            print(
                "[CAM]   WARNING: sin world_transform.npz; XYZ queda en coordenadas de cámara izquierda"
            )
    else:
        print("[CAM] Iniciando — SIN calibración estéreo")
    print(
        "[CAM] Para calibrar color del globo: pulsa 'P' y selecciona un rectángulo sobre él"
    )
    print(
        f"[CAM] HSV globo verde actual: {config.BALLOON_GREEN_HSV_LOWER.tolist()} → {config.BALLOON_GREEN_HSV_UPPER.tolist()}"
    )

    frame_count = 0

    while True:
        ret_l, frame_l = cap_l.read()
        ret_r, frame_r = cap_r.read()
        if not ret_l or not ret_r:
            time.sleep(0.001)
            continue

        # Tracking del globo verde
        with _hsv_lock:
            global _hsv_updated
            if _hsv_updated:
                tracker.reset()
                _hsv_updated = False
        det_l, det_r = tracker.update(frame_l, frame_r)

        if frame_count % 30 == 0:
            print(
                f"[DET] L={'✓' if det_l is not None else '✗'}  "
                f"R={'✓' if det_r is not None else '✗'}  "
                f"{'3D' if calibrated else 'NO-CAL'}"
            )

        # Triangulación estéreo con validación epipolar.
        pos_3d = None
        mode_label = None
        epipolar_error = None

        if calibrated and det_l is not None and det_r is not None:
            epipolar_error = reconstructor.epipolar_error_px(det_l[:2], det_r[:2])
            epipolar_ok = (
                epipolar_error is None or epipolar_error <= config.EPIPOLAR_MAX_ERROR_PX
            )
            if epipolar_ok:
                tri = reconstructor.triangulate_world(det_l[:2], det_r[:2])
                if tri is not None and np.isfinite(tri).all():
                    pos_3d = tri.copy()
                    mode_label = "WORLD" if reconstructor.has_world_transform else "CAM"
            elif frame_count % 15 == 0:
                print(
                    f"[STEREO] Correspondencia rechazada por error epipolar={epipolar_error:.2f}px"
                )

        if pos_3d is not None:
            with _balloon_lock:
                global _balloon_latest, _balloon_latest_ts
                _balloon_latest = pos_3d
                _balloon_latest_ts = time.time()
            if frame_count % 15 == 0:
                print(
                    f"[POS] {mode_label}: "
                    f"X={pos_3d[0]:6.1f}  Y={pos_3d[1]:6.1f}  Z={pos_3d[2]:6.1f}"
                )
            try:
                balloon_q.put_nowait(pos_3d)
            except queue.Full:
                try:
                    balloon_q.get_nowait()
                except queue.Empty:
                    pass
                try:
                    balloon_q.put_nowait(pos_3d)
                except queue.Full:
                    pass

        # Info en frames
        cal_status = "3D" if calibrated else "NO-CAL"
        for frame, det, name, tracker_side in [
            (frame_l, det_l, "L", tracker.left),
            (frame_r, det_r, "R", tracker.right),
        ]:
            if det is not None:
                tracker_side.draw_detection(frame, det)
                tracker_side.draw_trail(frame)
                cv2.putText(
                    frame,
                    "BALLOON",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
            else:
                cv2.putText(
                    frame,
                    "NO BALLOON",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )
            cv2.putText(
                frame,
                f"[{cal_status}] {name}",
                (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 200, 255),
                1,
            )
            if name == "L":
                draw_robot_ar_overlay(frame, reconstructor)
            if mode_label is not None:
                ep_txt = (
                    "" if epipolar_error is None else f" epi={epipolar_error:.1f}px"
                )
                cv2.putText(
                    frame,
                    mode_label + ep_txt,
                    (10, 78),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (200, 200, 0),
                    1,
                )
            if pos_3d is not None:
                cv2.putText(
                    frame,
                    f"X={pos_3d[0]:.1f} Y={pos_3d[1]:.1f} Z={pos_3d[2]:.1f}",
                    (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 0),
                    2,
                )

        try:
            left_frame_q.put_nowait(frame_l)
            right_frame_q.put_nowait(frame_r)
        except queue.Full:
            pass

        frame_count += 1


# ─────────────────────────────────────────────
# HILO GESTOS
# ─────────────────────────────────────────────
def _put_latest(q, item):
    try:
        q.put_nowait(item)
    except queue.Full:
        try:
            q.get_nowait()
        except queue.Empty:
            pass
        try:
            q.put_nowait(item)
        except queue.Full:
            pass


def gesture_worker(frame_q, command_q):
    gesture_controller_cls = None
    try:
        from src.gesture_robot import GestureRobotController

        gesture_controller_cls = GestureRobotController
    except Exception as e:
        print(f"[GESTURE] gesture_robot no disponible: {e}")

    if gesture_controller_cls is not None:
        controller = gesture_controller_cls(camera_index=config.CAMERA_GESTURE)
        if controller.start():
            print("[GESTURE] Activo")
            while True:
                frame = controller.get_frame()
                payload = controller.get_payload()
                if frame is not None:
                    _put_latest(frame_q, frame)
                if payload is not None:
                    _put_latest(command_q, payload)
                time.sleep(0.03)
            return
        else:
            print(
                "[GESTURE] GestureRobotController no pudo iniciarse; fallback a cámara en bruto"
            )

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
            cv2.putText(
                frame,
                "GESTURE CAM [Sin MediaPipe]",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )
            _put_latest(frame_q, frame)
        time.sleep(0.033)


# ─────────────────────────────────────────────
# CÁMARA PYBULLET
# ─────────────────────────────────────────────
class CameraController:
    MODES = ["cenital", "follow"]

    def __init__(self):
        self.mode_idx = 0
        self.smooth_pos = np.array([0.0, 0.0, 0.0])

    def get_mode(self):
        return self.MODES[self.mode_idx]

    def next_mode(self):
        self.mode_idx = (self.mode_idx + 1) % len(self.MODES)
        print(f"[CAM] Modo: {self.get_mode()}")

    def update(self, robot_pos):
        mode = self.get_mode()
        if mode == "cenital":
            p.resetDebugVisualizerCamera(18, 90, -89, robot_pos)
        elif mode == "follow":
            target = np.array(robot_pos)
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
        config.BALLOON_GREEN_HSV_LOWER = lower
        config.BALLOON_GREEN_HSV_UPPER = upper
        _hsv_updated = True
    return True


# ─────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────
def main():
    print("=" * 50)
    print("  BALLOON CATCH 3D - PyBullet + Stereo + Gestos")
    print("=" * 50)
    print("\nObjetivo:")
    print("  Lanza un globo verde real dentro del campo estéreo.")
    print(
        "  El sistema triangula XYZ solo con las dos cámaras y mueve el globo virtual."
    )
    print(
        "  Controla el robot con gestos y colócalo debajo cuando el globo toque el suelo."
    )
    print("\nControles de respaldo:")
    print("  W/S       : Avanzar / retroceder robot")
    print("  A/D       : Girar robot")
    print("  R         : Reset")
    print("  C         : Cambiar cámara virtual")
    print("  G         : Calibrar suelo con la altura estéreo actual del globo")
    print("  P         : Calibrar HSV del globo seleccionando ROI en Stereo Left")
    print("  Q / ESC   : Salir")
    print("\nIniciando...")

    _physics_client, robot_id = init_simulation()
    robot = RobotController(robot_id)
    cam = CameraController()
    balloon_id = create_tracked_balloon()

    # Lanzar hilos
    t_stereo = threading.Thread(
        target=stereo_worker,
        args=(
            config.CAMERA_LEFT,
            config.CAMERA_RIGHT,
            balloon_queue,
            stereo_left_queue,
            stereo_right_queue,
        ),
        daemon=True,
    )
    t_stereo.start()

    t_gesture = threading.Thread(
        target=gesture_worker, args=(gesture_queue, gesture_command_queue), daemon=True
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

    _balloon_pb_pos = [0.0, 0.0, BALLOON_RADIUS]
    _ground_z_cm = None
    _balloon_smooth = None
    _balloon_smooth_z = None
    _latest_gesture_payload = None
    score = 0
    attempts = 0
    last_touch_ts = 0.0
    was_touching_ground = False

    while running:
        now = time.time()
        dt = now - last_time
        last_time = now

        try:
            while True:
                _latest_gesture_payload = gesture_command_queue.get_nowait()
        except queue.Empty:
            pass

        _, robot_pos = robot.update(dt, _latest_gesture_payload)
        cam.update(robot_pos)
        with _robot_world_lock:
            global _robot_world_pose
            _robot_world_pose = pybullet_robot_to_world_cm(robot.x, robot.y)

        if key_pressed("q"):
            running = False
            continue

        if key_pressed("r") and not _r_was_down:
            robot.x = 0.0
            robot.y = -FIELD_LENGTH / 2 + 1.5
            robot.heading = 0.0
            p.resetBasePositionAndOrientation(
                robot_id, [robot.x, robot.y, 0.19], [0, 0, 0, 1]
            )
            print("[GAME] Reset")
        _r_was_down = key_pressed("r")

        if key_pressed("c") and not _c_was_down:
            cam.next_mode()
        _c_was_down = key_pressed("c")

        if key_pressed("g") and not _g_was_down:
            _g_request = True
            print("[BALLOON] Calibrando suelo: pon el globo en el suelo y pulsa G")
        _g_was_down = key_pressed("g")

        # --- Actualizar globo ---
        world_pos = None
        try:
            while True:
                world_pos = balloon_queue.get_nowait()
        except queue.Empty:
            pass

        if world_pos is None:
            with _balloon_lock:
                if (
                    _balloon_latest is not None
                    and (now - _balloon_latest_ts) < HOLD_LAST_SEC
                ):
                    world_pos = _balloon_latest

        z_bad = False
        x_cm = y_cm = z_cm = 0.0
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
            if (
                x_cm < -XY_MARGIN_CM
                or x_cm > config.PLAY_AREA_WIDTH + XY_MARGIN_CM
                or y_cm < -XY_MARGIN_CM
                or y_cm > config.PLAY_AREA_HEIGHT + XY_MARGIN_CM
            ):
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
                _balloon_smooth_z = None
                _g_request = False
                print(f"[BALLOON] Suelo calibrado manual: {_ground_z_cm:.1f} cm")

            if _ground_z_cm is None:
                _ground_z_cm = z_cm
                print(f"[BALLOON] Suelo inicial automático: {_ground_z_cm:.1f} cm")

            height_cm_raw = abs(z_cm - _ground_z_cm)
            if height_cm_raw < Z_DEADZONE_CM:
                height_cm_raw = 0.0
            if height_cm_raw < GROUND_LOCK_CM:
                _ground_z_cm = (
                    1.0 - GROUND_ADAPT_ALPHA
                ) * _ground_z_cm + GROUND_ADAPT_ALPHA * z_cm
            height_cm = min(height_cm_raw * Z_HEIGHT_GAIN, Z_MAX_HEIGHT_CM)

            x_pb = (x_cm / config.PLAY_AREA_WIDTH - 0.5) * FIELD_WIDTH
            y_pb = (y_cm / config.PLAY_AREA_HEIGHT - 0.5) * FIELD_LENGTH
            z_pb = BALLOON_RADIUS + height_cm * Z_SCALE

            if _balloon_smooth_z is None:
                _balloon_smooth_z = z_pb
            else:
                max_step = Z_MAX_STEP_CM * Z_SCALE
                delta_z = z_pb - _balloon_smooth_z
                if abs(delta_z) > max_step:
                    z_pb = _balloon_smooth_z + np.sign(delta_z) * max_step
                _balloon_smooth_z += (z_pb - _balloon_smooth_z) * 0.3
                z_pb = _balloon_smooth_z

            raw_pos = np.array([x_pb, y_pb, z_pb], dtype=np.float64)
            if _balloon_smooth is None:
                _balloon_smooth = raw_pos
            else:
                delta = raw_pos - _balloon_smooth
                if np.linalg.norm(delta) > BALLOON_MAX_JUMP:
                    raw_pos = _balloon_smooth + delta * 0.2
                _balloon_smooth = (
                    _balloon_smooth + (raw_pos - _balloon_smooth) * BALLOON_SMOOTH_ALPHA
                )

            _balloon_pb_pos = _balloon_smooth.tolist()
            _balloon_smooth_z = float(_balloon_smooth[2])
            p.resetBasePositionAndOrientation(balloon_id, _balloon_pb_pos, [0, 0, 0, 1])

            touching_ground = height_cm <= GROUND_TOUCH_HEIGHT_CM
            if (
                touching_ground
                and not was_touching_ground
                and (now - last_touch_ts) > CATCH_COOLDOWN_SEC
            ):
                attempts += 1
                robot_xy = np.array([robot.x, robot.y], dtype=np.float64)
                balloon_xy = np.array(_balloon_pb_pos[:2], dtype=np.float64)
                dist = float(np.linalg.norm(robot_xy - balloon_xy))
                caught = dist <= CATCH_RADIUS
                if caught:
                    score += 1
                    p.changeVisualShape(balloon_id, -1, rgbaColor=BALLOON_CAUGHT_COLOR)
                    print(
                        f"[CATCH] ¡Atrapado! score={score}/{attempts} dist={dist:.2f} m"
                    )
                else:
                    p.changeVisualShape(balloon_id, -1, rgbaColor=BALLOON_COLOR)
                    print(f"[CATCH] Fallo score={score}/{attempts} dist={dist:.2f} m")
                last_touch_ts = now
            elif not touching_ground:
                p.changeVisualShape(balloon_id, -1, rgbaColor=BALLOON_COLOR)
            was_touching_ground = touching_ground

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
                cv2.putText(
                    fl,
                    "ROI MODE",
                    (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                )
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
        if key == ord("p"):
            if last_left_frame is not None:
                roi = cv2.selectROI(
                    left_window, last_left_frame, fromCenter=False, showCrosshair=True
                )
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
