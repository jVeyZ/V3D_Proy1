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
from pathlib import Path

import cv2
import numpy as np
import pybullet as p
import pybullet_data
from pynput import keyboard as pynput_kb

import src.game_config as config

_PROJECT_ROOT = Path(__file__).resolve().parents[1]

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
_robot_world_pose = np.array([0.0, 0.0, 0.0], dtype=np.float64)

# ─────────────────────────────────────────────
# CONFIGURACIÓN
# ─────────────────────────────────────────────
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
HSV_MARGIN = np.array([12, 50, 50], dtype=np.int32)
BALLOON_SMOOTH_ALPHA = 0.35
BALLOON_MAX_JUMP = 1.2
Z_DEADZONE_CM = 1.5
Z_MAX_STEP_CM = 3.0
HOLD_LAST_SEC = 1.5
Z_HEIGHT_GAIN = 2.0
GROUND_LOCK_CM = 2.0
GROUND_ADAPT_ALPHA = 0.15


# ─────────────────────────────────────────────
# INICIALIZACIÓN PYBULLET
# ─────────────────────────────────────────────
def init_simulation():
    use_gui = os.environ.get("PYBULLET_DIRECT") not in ("1", "true", "TRUE")
    physics_client = p.connect(p.GUI if use_gui else p.DIRECT)
    p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.resetDebugVisualizerCamera(15, 60, -30, [0, 0, 0])

    # ── Open arena: suelo enorme sin límites ──
    ground_half = 50.0
    plane_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[ground_half, ground_half, 0.01])
    plane_visual = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[ground_half, ground_half, 0.01],
        rgbaColor=[0.15, 0.35, 0.15, 1.0], specularColor=[0.05, 0.05, 0.05])
    p.createMultiBody(0, plane_shape, plane_visual, basePosition=[0, 0, -0.01])

    # ── Marcas de referencia en el suelo cada 1 m ──
    for i in range(-40, 41, 10):
        si = i * config.WORLD_SCALE
        half_extent = 40 * config.WORLD_SCALE
        p.addUserDebugLine([-half_extent, si, 0.01],
                           [half_extent, si, 0.01],
                           [0.3, 0.3, 0.3], 1, lifeTime=0)
        p.addUserDebugLine([si, -half_extent, 0.01],
                           [si, half_extent, 0.01],
                           [0.3, 0.3, 0.3], 1, lifeTime=0)

    robot_id = create_robot()
    p.changeVisualShape(robot_id, -1, rgbaColor=ROBOT_COLOR)
    return physics_client, robot_id


def create_robot():
    """Crea el robot/cesta digital.

    Si existe assets/Robot.obj se usa como malla visual. Si no, se usa una caja
    PyBullet nativa para que la demo sea autocontenida.
    """
    mesh_path = str(_PROJECT_ROOT / "assets" / "Robot.obj")

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
        basePosition=[0, 0.0, ROBOT_Z_CENTER],
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
_ARROW_MAP = {
    pynput_kb.Key.up: "ARROW_UP",
    pynput_kb.Key.down: "ARROW_DOWN",
    pynput_kb.Key.left: "ARROW_LEFT",
    pynput_kb.Key.right: "ARROW_RIGHT",
}


def _on_press(key):
    try:
        _keys_down.add(key.char)
    except AttributeError:
        name = _ARROW_MAP.get(key)
        if name:
            _keys_down.add(name)
        else:
            _keys_down.add(key)


def _on_release(key):
    try:
        _keys_down.discard(key.char)
    except AttributeError:
        name = _ARROW_MAP.get(key)
        if name:
            _keys_down.discard(name)
        else:
            _keys_down.discard(key)


_kb_listener = pynput_kb.Listener(on_press=_on_press, on_release=_on_release)


def key_pressed(c):
    return c in _keys_down


def pybullet_robot_to_world_cm(robot_x, robot_y):
    """Convierte posición PyBullet del robot a coordenadas ChArUco (cm) para proyección AR.

    El globo se triangula en coordenadas ChArUco, luego se aplica WORLD_SWAP_XY/FLIP
    para pasarlas al sistema de juego antes de mapear a PyBullet. Para la proyección
    inversa (robot → píxeles de cámara) hay que deshacer esos mismos transforms.
    """
    world_x = robot_x / config.WORLD_SCALE
    world_y = robot_y / config.WORLD_SCALE
    if config.WORLD_FLIP_Y:
        world_y = -world_y
    if config.WORLD_FLIP_X:
        world_x = -world_x
    if config.WORLD_SWAP_XY:
        world_x, world_y = world_y, world_x
    return np.array([world_x, world_y, 0.0], dtype=np.float64)


def draw_robot_ar_overlay(frame, reconstructor, project_fn, label="ROBOT"):
    """Proyecta la huella del robot virtual sobre el frame de cámara."""
    if not reconstructor.has_world_transform:
        return
    with _robot_world_lock:
        center = _robot_world_pose.copy()

    half_x_cm = ROBOT_HALF_EXTENTS[0] / config.WORLD_SCALE
    half_y_cm = ROBOT_HALF_EXTENTS[1] / config.WORLD_SCALE
    corners_world = [
        center + np.array([-half_x_cm, -half_y_cm, 0.0]),
        center + np.array([half_x_cm, -half_y_cm, 0.0]),
        center + np.array([half_x_cm, half_y_cm, 0.0]),
        center + np.array([-half_x_cm, half_y_cm, 0.0]),
    ]
    projected = [project_fn(pt) for pt in corners_world]
    if any(pt is None for pt in projected):
        return
    pts = np.array(projected, dtype=np.int32).reshape((-1, 1, 2))
    cv2.polylines(frame, [pts], isClosed=True, color=(255, 80, 40), thickness=2)
    center_px = project_fn(center)
    if center_px is not None:
        cv2.circle(frame, center_px, 5, (255, 80, 40), -1)
        cv2.putText(
            frame,
            label,
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
        self.y = 0.0
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
def stereo_worker(cap_l, cap_r, balloon_q, left_frame_q, right_frame_q):
    from src.balloon_tracker import StereoGreenBalloonTracker
    from src.stereo import Stereo3DReconstructor

    cap_l.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
    cap_l.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap_r.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
    cap_r.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)

    actual_w_l = int(cap_l.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h_l = int(cap_l.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_w_r = int(cap_r.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h_r = int(cap_r.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[CAM] Solicitado: {config.CAMERA_WIDTH}x{config.CAMERA_HEIGHT}")
    print(f"[CAM] Left real:  {actual_w_l}x{actual_h_l} | Right real: {actual_w_r}x{actual_h_r}")

    target_w = config.CAMERA_WIDTH
    target_h = config.CAMERA_HEIGHT
    if (actual_w_l, actual_h_l) != (target_w, target_h) or (actual_w_r, actual_h_r) != (target_w, target_h):
        print(f"[CAM] AVISO: al menos una camara no soporta la resolucion pedida. "
              f"Forzando resize a {target_w}x{target_h} en todas.")

    tracker = StereoGreenBalloonTracker()
    reconstructor = Stereo3DReconstructor()
    calib_path = str(_PROJECT_ROOT / "calibration" / "stereo_charuco.npz")
    world_transform_path = str(
        _PROJECT_ROOT / "calibration" / "world_transform.npz"
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
            world_imgs = _PROJECT_ROOT / "calibration" / "world_charuco"
            reconstructor.refine_fundamental_from_floor(
                str(world_imgs / "left" / "left_000.png"),
                str(world_imgs / "right" / "right_000.png"),
            )

    calibrated = reconstructor.is_calibrated
    if calibrated:
        calib_size = reconstructor.image_size
        if calib_size is not None and calib_size != (target_w, target_h):
            print(
                f"[CAM] *** ERROR: resolucion runtime ({target_w}x{target_h}) "
                f"no coincide con calibracion ({calib_size[0]}x{calib_size[1]}). "
                f"La triangulacion sera incorrecta. ***"
            )
            print(
                "[CAM]   Solucion: recalibra a la resolucion actual o cambia "
                "CAMERA_WIDTH/HEIGHT en game_config.py para que coincida."
            )
        print("[CAM] Iniciando — calibración estéreo cargada (sin ArUco en runtime)")
        print(f"[CAM]   K_left=\n{reconstructor.K_left}")
        print(f"[CAM]   K_right=\n{reconstructor.K_right}")
        if reconstructor.has_world_transform:
            print("[CAM]   Sistema de mundo: calibration/world_transform.npz")
            rp_world = np.array([0.0, 0.0, 0.0], dtype=np.float64)
            pl = reconstructor.project_world_to_left(rp_world)
            pr = reconstructor.project_world_to_right(rp_world)
            if pl and pr:
                F = reconstructor.F_refined if reconstructor.F_refined is not None else reconstructor.F
                if F is not None:
                    line = F @ np.array([pl[0], pl[1], 1.0], dtype=np.float64)
                    a, b, c_ = line[0], line[1], line[2]
                    dist = abs(a * pr[0] + b * pr[1] + c_) / np.hypot(a, b)
                    print(f"[CAM]   AR consistency: robot proj L={pl} R={pr} epi-dist={dist:.1f}px")
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

        h_l, w_l = frame_l.shape[:2]
        h_r, w_r = frame_r.shape[:2]
        if (w_l, h_l) != (target_w, target_h):
            frame_l = cv2.resize(frame_l, (target_w, target_h))
        if (w_r, h_r) != (target_w, target_h):
            frame_r = cv2.resize(frame_r, (target_w, target_h))

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
            snap_l, snap_r = reconstructor.snap_to_epipolar(det_l[:2], det_r[:2])
            epipolar_error = reconstructor.epipolar_error_px(snap_l, snap_r)
            if frame_count == 1:
                corr_l = np.hypot(snap_l[0] - det_l[0], snap_l[1] - det_l[1])
                corr_r = np.hypot(snap_r[0] - det_r[0], snap_r[1] - det_r[1])
                print(f"[STEREO] Epipolar correction: L={corr_l:.1f}px R={corr_r:.1f}px")
            tri = reconstructor.triangulate_world(snap_l, snap_r)
            if tri is not None and np.isfinite(tri).all():
                pos_3d = tri.copy()
                if config.WORLD_SWAP_XY:
                    pos_3d[0], pos_3d[1] = pos_3d[1], pos_3d[0]
                if config.WORLD_FLIP_X:
                    pos_3d[0] = -pos_3d[0]
                if config.WORLD_FLIP_Y:
                    pos_3d[1] = -pos_3d[1]
                if config.WORLD_FLIP_Z:
                    pos_3d[2] = -pos_3d[2]
                mode_label = "WORLD" if reconstructor.has_world_transform else "CAM"

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
                draw_robot_ar_overlay(frame, reconstructor,
                                      reconstructor.project_world_to_left, "ROBOT AR")
            else:
                draw_robot_ar_overlay(frame, reconstructor,
                                      reconstructor.project_world_to_right, "ROBOT AR")
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

        # ── Epipolar debug: linea de la deteccion del otro lado ──
        if calibrated and det_l is not None and det_r is not None:
            F = reconstructor.F_refined if reconstructor.F_refined is not None else reconstructor.F
            if F is not None:
                l_from_left = F @ np.array([det_l[0], det_l[1], 1.0], dtype=np.float64)
                l_from_right = F.T @ np.array([det_r[0], det_r[1], 1.0], dtype=np.float64)
                h_l, w_l = frame_l.shape[:2]
                h_r, w_r = frame_r.shape[:2]
                for (frame, line, name, w, h) in [(frame_r, l_from_left, "R", w_r, h_r),
                                                    (frame_l, l_from_right, "L", w_l, h_l)]:
                    a, b, c = line[0], line[1], line[2]
                    if abs(b) > 1e-6:
                        x0, y0 = 0, int(-c / b)
                        x1, y1 = w - 1, int(-(a * (w - 1) + c) / b)
                    elif abs(a) > 1e-6:
                        x0, y0 = int(-c / a), 0
                        x1, y1 = int(-(b * (h - 1) + c) / a), h - 1
                    else:
                        continue
                    cv2.line(frame, (x0, y0), (x1, y1), (255, 0, 255), 1, cv2.LINE_AA)

        # ── Proyectar grid de suelo en ambos frames ──
        if reconstructor.has_world_transform and frame_count % 30 == 0:
            for gx in range(-20, 100, 20):
                for gy in range(-20, 100, 20):
                    for (frame, proj_fn) in [(frame_l, reconstructor.project_world_to_left),
                                              (frame_r, reconstructor.project_world_to_right)]:
                        p = proj_fn(np.array([gx, gy, 0.0]))
                        if p is not None and 0 <= p[0] < frame.shape[1] and 0 <= p[1] < frame.shape[0]:
                            cv2.circle(frame, p, 1, (0, 255, 0), -1)

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


def gesture_worker(cap, frame_q, command_q):
    gesture_controller_cls = None
    try:
        from src.gesture_robot import GestureRobotController

        gesture_controller_cls = GestureRobotController
    except Exception as e:
        print(f"[GESTURE] gesture_robot no disponible: {e}")

    if gesture_controller_cls is not None:
        model_path = str(_PROJECT_ROOT / "assets" / "gesture_recognizer.task")
        controller = gesture_controller_cls(camera_index=config.CAMERA_GESTURE, camera=cap, model_path=model_path)
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
    MODES = ["free", "cenital", "follow"]

    def __init__(self):
        self.mode_idx = 0
        self.smooth_pos = np.array([0.0, 0.0, 0.0])
        self.dist = 15.0
        self.yaw = 60.0
        self.pitch = -30.0
        self.target = np.array([0.0, 0.0, 0.0])

    def get_mode(self):
        return self.MODES[self.mode_idx]

    def next_mode(self):
        self.mode_idx = (self.mode_idx + 1) % len(self.MODES)
        print(f"[CAM] Modo: {self.get_mode()}")

    def update(self, robot_pos):
        mode = self.get_mode()
        target = np.array(robot_pos)
        self.smooth_pos += (target - self.smooth_pos) * 0.12

        if mode == "free":
            self.target = self.smooth_pos
            p.resetDebugVisualizerCamera(self.dist, self.yaw, self.pitch, self.target.tolist())
        elif mode == "cenital":
            p.resetDebugVisualizerCamera(18, 90, -89, robot_pos)
        elif mode == "follow":
            p.resetDebugVisualizerCamera(6, 180, -30, self.smooth_pos.tolist())


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
    print("  C         : Cambiar cámara virtual (free → cenital → follow)")
    print("  Free cam  : Flechas=rotar  Z/X=zoom")
    print("  G         : Calibrar suelo con la altura estéreo actual del globo")
    print("  P         : Calibrar HSV del globo seleccionando ROI en Stereo Left")
    print("  Q / ESC   : Salir")
    print("\nIniciando...")

    # Arrancar listener de teclado
    _kb_listener.start()

    # Abrir cámaras en el hilo principal (obligatorio en macOS)
    print("[CAM] Abriendo cámaras...")
    cap_l = cv2.VideoCapture(config.CAMERA_LEFT)
    cap_r = cv2.VideoCapture(config.CAMERA_RIGHT)
    cap_g = cv2.VideoCapture(config.CAMERA_GESTURE)
    if not cap_l.isOpened():
        print(f"[CAM] ERROR: No se pudo abrir cámara izquierda ({config.CAMERA_LEFT})")
    if not cap_r.isOpened():
        print(f"[CAM] ERROR: No se pudo abrir cámara derecha ({config.CAMERA_RIGHT})")
    if not cap_g.isOpened():
        print(f"[CAM] ERROR: No se pudo abrir cámara de gestos ({config.CAMERA_GESTURE})")

    _physics_client, robot_id = init_simulation()
    robot = RobotController(robot_id)
    cam = CameraController()
    balloon_id = create_tracked_balloon()

    # Lanzar hilos
    t_stereo = threading.Thread(
        target=stereo_worker,
        args=(cap_l, cap_r, balloon_queue, stereo_left_queue, stereo_right_queue),
        daemon=True,
    )
    t_stereo.start()

    t_gesture = threading.Thread(
        target=gesture_worker,
        args=(cap_g, gesture_queue, gesture_command_queue),
        daemon=True,
    )
    t_gesture.start()

    running = True
    last_time = time.time()
    _r_was_down = False
    _c_was_down = False
    _p_was_down = False
    _g_was_down = False
    _g_request = False

    left_window = "Stereo Left"
    right_window = "Stereo Right"
    left_window_ready = False
    right_window_ready = False
    last_left_frame = None

    _balloon_pb_pos = [0.0, 0.0, BALLOON_RADIUS]
    _ground_z_cm = None
    _balloon_smooth = None
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
            robot.y = 0.0
            robot.heading = 0.0
            q_z = p.getQuaternionFromEuler([0, 0, np.pi])
            q_x = p.getQuaternionFromEuler([np.pi / 2, 0, 0])
            _, base_orn = p.multiplyTransforms([0, 0, 0], q_z, [0, 0, 0], q_x)
            p.resetBasePositionAndOrientation(
                robot_id, [robot.x, robot.y, ROBOT_Z_CENTER], base_orn
            )
            print("[GAME] Reset")
        _r_was_down = key_pressed("r")

        if key_pressed("c") and not _c_was_down:
            cam.next_mode()
        _c_was_down = key_pressed("c")

        # ── Controles de cámara libre ──
        if cam.get_mode() == "free":
            speed = 120.0 * dt
            if key_pressed("ARROW_LEFT"):
                cam.yaw -= speed
            if key_pressed("ARROW_RIGHT"):
                cam.yaw += speed
            if key_pressed("ARROW_UP"):
                cam.pitch = min(0, cam.pitch + speed)
            if key_pressed("ARROW_DOWN"):
                cam.pitch = max(-89, cam.pitch - speed)
            if key_pressed("z"):
                cam.dist = max(1, cam.dist - speed * 5)
            if key_pressed("x"):
                cam.dist = min(80, cam.dist + speed * 5)
            cam.yaw %= 360

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
            if abs(z_cm) > 1000.0:
                z_bad = True

        if world_pos is not None:
            if z_bad:
                if _ground_z_cm is not None:
                    z_cm = _ground_z_cm
                else:
                    z_cm = 0.0

            if _g_request:
                _ground_z_cm = z_cm
                _balloon_smooth = None
                _g_request = False
                print(f"[BALLOON] Suelo calibrado manual: {_ground_z_cm:.1f} cm")

            if _ground_z_cm is None:
                _ground_z_cm = z_cm
                print(f"[BALLOON] Suelo inicial automático: {_ground_z_cm:.1f} cm")
                print("[BALLOON]   Asegúrate de que el globo está en el suelo. "
                      "Si no, pulsa G con el globo en el suelo para recalibrar.")

            height_cm_raw = abs(z_cm - _ground_z_cm)
            if height_cm_raw < Z_DEADZONE_CM:
                height_cm_raw = 0.0
            if height_cm_raw < GROUND_LOCK_CM:
                _ground_z_cm = (
                    1.0 - GROUND_ADAPT_ALPHA
                ) * _ground_z_cm + GROUND_ADAPT_ALPHA * z_cm
            height_cm = height_cm_raw * Z_HEIGHT_GAIN

            x_pb = x_cm * config.WORLD_SCALE
            y_pb = y_cm * config.WORLD_SCALE
            z_pb = BALLOON_RADIUS + height_cm * config.WORLD_SCALE

            if _balloon_smooth is not None:
                max_step = Z_MAX_STEP_CM * config.WORLD_SCALE
                prev_z = float(_balloon_smooth[2])
                delta_z = z_pb - prev_z
                if abs(delta_z) > max_step:
                    z_pb = prev_z + np.sign(delta_z) * max_step
                z_pb = prev_z + (z_pb - prev_z) * 0.3

            raw_pos = np.array([x_pb, y_pb, z_pb], dtype=np.float64)
            if _balloon_smooth is None:
                _balloon_smooth = raw_pos
            else:
                delta = raw_pos - _balloon_smooth
                delta_norm = float(np.linalg.norm(delta))
                if delta_norm > BALLOON_MAX_JUMP:
                    raw_pos = _balloon_smooth + delta * (BALLOON_MAX_JUMP / delta_norm)
                _balloon_smooth = (
                    _balloon_smooth + (raw_pos - _balloon_smooth) * BALLOON_SMOOTH_ALPHA
                )

            _balloon_pb_pos = _balloon_smooth.tolist()
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
                left_window_ready = True
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

    _kb_listener.stop()
    p.disconnect()
    cv2.destroyAllWindows()
    print("\n[GAME] Cerrando...")


if __name__ == "__main__":
    main()
