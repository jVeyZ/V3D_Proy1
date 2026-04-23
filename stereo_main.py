"""Aplicacion mini-golf con reconstruccion 3D usando dos camaras."""

import argparse
import time

import cv2
import numpy as np

import config
from ar_viewer import ARViewer
from calibration import HomographyCalibrator
from detection import AutomaticDetector
from game_engine import MiniGolfEngine
from stereo import Stereo3DReconstructor
from tracking import OpenCVTracker, create_tracker
from virtual_scene import SimpleVirtualScene, VirtualScene


def parse_args():
    parser = argparse.ArgumentParser(
        description="Mini-golf virtual estereo (dos camaras + triangulacion 3D)"
    )
    parser.add_argument(
        "--camera-left",
        type=str,
        default=str(config.CAMERA_LEFT_SOURCE),
        help="Fuente camara izquierda (indice o URL)",
    )
    parser.add_argument(
        "--camera-right",
        type=str,
        default=str(config.CAMERA_RIGHT_SOURCE),
        help="Fuente camara derecha (indice o URL)",
    )
    parser.add_argument(
        "--tracker",
        choices=["color", "csrt", "kcf", "mosse"],
        default=config.TRACKING_METHOD,
        help=f"Metodo de seguimiento (por defecto {config.TRACKING_METHOD})",
    )
    parser.add_argument(
        "--no-3d",
        action="store_true",
        help="Desactiva visualizacion Open3D",
    )
    parser.add_argument(
        "--fov-deg",
        type=float,
        default=config.STEREO_DEFAULT_FOV_DEG,
        help="FOV horizontal aproximado para ambos moviles",
    )
    parser.add_argument(
        "--recalibrate-every",
        type=int,
        default=config.STEREO_RECALIBRATE_EVERY_N_FRAMES,
        help="Recalibrar pose estereo cada N frames",
    )
    return parser.parse_args()


def _parse_camera_source(src_text):
    try:
        return int(src_text)
    except Exception:
        return src_text


class StereoMiniGolfApp:
    def __init__(self, args):
        self.args = args

        self.left_source = _parse_camera_source(args.camera_left)
        self.right_source = _parse_camera_source(args.camera_right)
        self.reconnect_every_frames = 30
        self._last_reconnect_frame = -9999
        # Forzado por usuario: mostrar guia en la camara derecha.
        self.guide_on_side = "right"

        self.cap_left = None
        self.cap_right = None

        self.left_calibrator = HomographyCalibrator()
        self.ar_viewer = ARViewer(self.left_calibrator)

        self.detector_left = AutomaticDetector()
        self.detector_right = AutomaticDetector()
        self.tracker_left = create_tracker(args.tracker)
        self.tracker_right = create_tracker(args.tracker)

        self.stereo = Stereo3DReconstructor(fov_deg=args.fov_deg)

        self.game_engine = MiniGolfEngine()
        self.virtual_scene_2d = SimpleVirtualScene()
        self.virtual_scene_3d = None if args.no_3d else VirtualScene()

        self.first_detection = True
        self.show_help = False
        self.running = True
        self.show_placement_guide = True

        self.ball_pos_3d = None

    def run(self):
        print("\n" + "=" * 68)
        print(" MINI-GOLF ESTEREO - Dos camaras + triangulacion 3D")
        print("=" * 68)

        self._init_cameras()

        if self.virtual_scene_3d is not None:
            if not self.virtual_scene_3d.initialize(
                hole_position=self.game_engine.state.hole_position
            ):
                self.virtual_scene_3d = None

        self.game_engine.start_game()
        self._main_loop()
        self._cleanup()

    def _init_cameras(self):
        self.cap_left = self._open_camera(self.left_source, "izquierda")
        self.cap_right = self._open_camera(self.right_source, "derecha")
        if self.cap_left is None or self.cap_right is None:
            print("[Stereo] Iniciando en modo espera: falta al menos una camara.")
            print("[Stereo] La app seguira abierta y reintentara conectar automaticamente.")

    def _open_camera(self, source, label):
        print(f"[Stereo] Abriendo camara {label}: {source}")
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            cap.release()
            print(f"[Stereo] Camara {label} no disponible por ahora.")
            return None

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
        print(f"[Stereo] Camara {label} conectada.")
        return cap

    def _placeholder_frame(self, title, subtitle):
        frame = np.zeros((config.CAMERA_HEIGHT, config.CAMERA_WIDTH, 3), dtype=np.uint8)
        cv2.putText(
            frame,
            title,
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 200, 255),
            2,
        )
        cv2.putText(
            frame,
            subtitle,
            (20, 95),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (230, 230, 230),
            1,
        )
        cv2.putText(
            frame,
            "Pulsa q para salir.",
            (20, config.CAMERA_HEIGHT - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (180, 180, 180),
            1,
        )
        return frame

    def _read_side(self, side):
        if side == "left":
            cap = self.cap_left
            label = "izquierda"
        else:
            cap = self.cap_right
            label = "derecha"

        if cap is None:
            return False, self._placeholder_frame(
                f"Camara {label} desconectada", "Esperando conexion..."
            )

        ok, frame = cap.read()
        if not ok or frame is None:
            cap.release()
            if side == "left":
                self.cap_left = None
            else:
                self.cap_right = None
            return False, self._placeholder_frame(
                f"Camara {label} desconectada", "Reintentando automaticamente..."
            )

        return True, frame

    def _attempt_reconnect_missing(self, frame_idx):
        if frame_idx - self._last_reconnect_frame < self.reconnect_every_frames:
            return
        self._last_reconnect_frame = frame_idx

        reconnected = False

        if self.cap_left is None:
            self.cap_left = self._open_camera(self.left_source, "izquierda")
            reconnected = reconnected or (self.cap_left is not None)

        if self.cap_right is None:
            self.cap_right = self._open_camera(self.right_source, "derecha")
            reconnected = reconnected or (self.cap_right is not None)

        if reconnected:
            self.first_detection = True
            self.stereo.reset()
            print("[Stereo] Conexion de camara actualizada. Reiniciando deteccion/calibracion.")

    def _main_loop(self):
        fps_timer = time.time()
        frame_count = 0
        fps_display = 0.0

        recalib_n = max(1, int(self.args.recalibrate_every))
        frame_idx = 0

        while self.running:
            frame_idx += 1
            self._attempt_reconnect_missing(frame_idx)

            ok_l, frame_left = self._read_side("left")
            ok_r, frame_right = self._read_side("right")

            if not (ok_l and ok_r):
                self.ball_pos_3d = None

                hole_pos = self.game_engine.state.hole_position
                self.virtual_scene_2d.update_hole(hole_pos)
                if self.virtual_scene_3d is not None:
                    self.virtual_scene_3d.update_hole_position(hole_pos)
                    self.virtual_scene_3d.step()

                left_view = self._render_left_view(frame_left, None, fps_display)
                right_view = self._render_right_view(frame_right, None, ok_r)
                cv2.imshow("Mini-Golf AR (Izquierda)", left_view)
                cv2.imshow("Camara Derecha", right_view)
                cv2.imshow("Gemelo Digital 2D", self.virtual_scene_2d.render())

                key = cv2.waitKey(1) & 0xFF
                if not self._handle_key(key):
                    break
                continue

            # Calibracion de homografia izquierda (para overlay AR)
            if frame_idx % 5 == 0 or not self.left_calibrator.is_calibrated:
                self.left_calibrator.calibrate_aruco(frame_left, quiet=True)

            # Recalibracion estereo periodica para soportar ligeros movimientos.
            if frame_idx % recalib_n == 0 or not self.stereo.is_calibrated:
                self.stereo.calibrate_from_aruco(frame_left, frame_right)

            det_left, det_right = self._detect_or_track(frame_left, frame_right)

            if det_left is not None and det_right is not None and self.stereo.is_calibrated:
                self.ball_pos_3d = self.stereo.triangulate(
                    (det_left[0], det_left[1]),
                    (det_right[0], det_right[1]),
                )
            else:
                self.ball_pos_3d = None

            ball_world_pos = None
            if self.ball_pos_3d is not None:
                ball_world_pos = np.array([self.ball_pos_3d[0], self.ball_pos_3d[1]])

            self.game_engine.update(ball_world_pos)

            if ball_world_pos is not None:
                self.virtual_scene_2d.update_ball(ball_world_pos)
                if self.virtual_scene_3d is not None:
                    self.virtual_scene_3d.update_ball_position(ball_world_pos)

                if self.left_calibrator.is_calibrated:
                    img_pos = self.left_calibrator.world_to_image(ball_world_pos)
                    if img_pos is not None:
                        self.ar_viewer.add_trail_point(img_pos)

            hole_pos = self.game_engine.state.hole_position
            self.virtual_scene_2d.update_hole(hole_pos)
            if self.virtual_scene_3d is not None:
                self.virtual_scene_3d.update_hole_position(hole_pos)
                self.virtual_scene_3d.step()

            left_view = self._render_left_view(frame_left, det_left, fps_display)
            right_view = self._render_right_view(frame_right, det_right, ok_r)
            cv2.imshow("Mini-Golf AR (Izquierda)", left_view)
            cv2.imshow("Camara Derecha", right_view)
            cv2.imshow("Gemelo Digital 2D", self.virtual_scene_2d.render())

            frame_count += 1
            elapsed = time.time() - fps_timer
            if elapsed > 1.0:
                fps_display = frame_count / elapsed
                fps_timer = time.time()
                frame_count = 0

            key = cv2.waitKey(1) & 0xFF
            if not self._handle_key(key):
                break

    def _detect_or_track(self, frame_left, frame_right):
        if self.first_detection:
            det_left = self.detector_left.detect(frame_left)
            det_right = self.detector_right.detect(frame_right)

            if det_left is None or det_right is None:
                return None, None

            self._init_trackers(det_left, det_right, frame_left, frame_right)
            self.first_detection = False
            return det_left, det_right

        det_left = self.tracker_left.update(frame_left)
        det_right = self.tracker_right.update(frame_right)

        lost_left = det_left is None or (
            hasattr(self.tracker_left, "is_lost") and self.tracker_left.is_lost
        )
        lost_right = det_right is None or (
            hasattr(self.tracker_right, "is_lost") and self.tracker_right.is_lost
        )

        if lost_left or lost_right:
            red_left = self.detector_left.detect(frame_left)
            red_right = self.detector_right.detect(frame_right)
            if red_left is not None and red_right is not None:
                self._init_trackers(red_left, red_right, frame_left, frame_right)
                return red_left, red_right
            return None, None

        return det_left, det_right

    def _init_trackers(self, det_left, det_right, frame_left, frame_right):
        self.tracker_left = create_tracker(self.args.tracker)
        self.tracker_right = create_tracker(self.args.tracker)

        if isinstance(self.tracker_left, OpenCVTracker):
            self.tracker_left.initialize(det_left[0], det_left[1], det_left[2], frame_left)
        else:
            self.tracker_left.initialize(det_left[0], det_left[1], det_left[2])

        if isinstance(self.tracker_right, OpenCVTracker):
            self.tracker_right.initialize(
                det_right[0], det_right[1], det_right[2], frame_right
            )
        else:
            self.tracker_right.initialize(det_right[0], det_right[1], det_right[2])

    def _render_left_view(self, frame_left, det_left, fps_value):
        if self.left_calibrator.is_calibrated:
            output = self.ar_viewer.draw(
                frame_left,
                game_state=self.game_engine.get_game_state_dict(),
                ball_detection=det_left,
            )
        else:
            output = frame_left.copy()
            if det_left is not None:
                self._draw_detection(output, det_left)

        if self.show_placement_guide and self.guide_on_side == "left":
            self._draw_mobile_reference_guide(output)

        self._draw_stereo_status(output)
        cv2.putText(
            output,
            f"FPS: {fps_value:.0f}",
            (10, output.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (220, 220, 220),
            1,
        )

        if self.show_help:
            self._draw_help(output)

        return output

    def _render_right_view(self, frame_right, det_right, right_connected=True):
        output = frame_right.copy()
        if det_right is not None:
            self._draw_detection(output, det_right)

        if self.show_placement_guide and self.guide_on_side == "right":
            self._draw_mobile_reference_guide(output)

        cv2.putText(
            output,
            f"GUIA: {'ON' if self.show_placement_guide else 'OFF'}",
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0) if self.show_placement_guide else (0, 140, 255),
            2,
        )

        cv2.putText(
            output,
            "Camara derecha (tracking)" if right_connected else "Camara derecha en espera",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
        return output

    def _draw_detection(self, frame, detection):
        cx, cy, r = detection
        cv2.circle(frame, (int(cx), int(cy)), int(r), (0, 255, 0), 2)
        cv2.circle(frame, (int(cx), int(cy)), 3, (0, 0, 255), -1)

    def _draw_mobile_reference_guide(self, frame):
        """Dibuja una guia verde rectangular sobre la camara del Mac.

        Representa la posicion esperada del movil en el extremo opuesto.
        """
        h, w = frame.shape[:2]
        # Rectangulo mas horizontal (ratio ancho/alto = 1.8), bajo el centro.
        ratio = 1.8
        rect_w = int(w * 0.13)
        rect_h = int(rect_w / ratio)

        cx = w // 2
        x0 = cx - rect_w // 2
        # Casi tocando borde inferior
        bottom_margin = max(4, int(h * 0.015))
        y1 = h - bottom_margin
        y0 = y1 - rect_h
        x1 = x0 + rect_w
        y1 = y0 + rect_h

        x0 = max(0, x0)
        y0 = max(0, y0)
        x1 = min(w - 1, x1)
        y1 = min(h - 1, y1)

        overlay = frame.copy()
        cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 120, 0), -1)
        cv2.addWeighted(overlay, 0.16, frame, 0.84, 0, frame)
        cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0), 3)

        cv2.putText(
            frame,
            "Referencia movil",
            (max(10, x0 - 5), max(20, y0 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
        )

    def _draw_stereo_status(self, frame):
        y = 26

        left_connected = self.cap_left is not None
        right_connected = self.cap_right is not None

        txt_cam = (
            f"CAM L:{'OK' if left_connected else 'OFF'} "
            f"CAM R:{'OK' if right_connected else 'OFF'}"
        )
        col_cam = (0, 255, 0) if (left_connected and right_connected) else (0, 200, 255)
        cv2.putText(frame, txt_cam, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col_cam, 2)
        y += 26

        text_stereo = (
            "ESTEREO: OK" if self.stereo.is_calibrated else "ESTEREO: esperando ArUco"
        )
        color_stereo = (0, 255, 0) if self.stereo.is_calibrated else (0, 200, 255)
        cv2.putText(frame, text_stereo, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_stereo, 2)
        y += 26

        text_h = (
            "AR izquierda: OK"
            if self.left_calibrator.is_calibrated
            else "AR izquierda: esperando ArUco"
        )
        color_h = (0, 255, 0) if self.left_calibrator.is_calibrated else (0, 200, 255)
        cv2.putText(frame, text_h, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_h, 2)
        y += 26

        guide_side_txt = "L" if self.guide_on_side == "left" else "R"
        guide_txt = f"GUIA: {'ON' if self.show_placement_guide else 'OFF'} ({guide_side_txt})"
        cv2.putText(frame, guide_txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if self.show_placement_guide else (0, 160, 255), 2)
        y += 26

        if self.ball_pos_3d is not None:
            x, y3, z = self.ball_pos_3d
            cv2.putText(
                frame,
                f"3D (cm): X={x:5.1f} Y={y3:5.1f} Z={z:5.1f}",
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (255, 255, 255),
                2,
            )

    def _draw_help(self, frame):
        overlay = frame.copy()
        h, w = frame.shape[:2]
        x0, y0 = int(w * 0.08), int(h * 0.12)
        x1, y1 = int(w * 0.92), int(h * 0.58)

        cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)

        lines = [
            "Controles:",
            "q / ESC: salir",
            "h: mostrar/ocultar ayuda",
            "g: mostrar/ocultar guia de posicion movil",
            "v: activar/desactivar escena 3D",
            "f: forzar re-deteccion",
            "c: recalibrar estereo + AR",
            "n: nueva partida",
            "r: reiniciar hoyo actual",
        ]

        y = y0 + 32
        for line in lines:
            cv2.putText(
                frame,
                line,
                (x0 + 20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65 if line == "Controles:" else 0.55,
                (0, 255, 255) if line == "Controles:" else (255, 255, 255),
                2 if line == "Controles:" else 1,
            )
            y += 28

    def _handle_key(self, key):
        if key == 255:
            return True

        if key in (ord("q"), 27):
            print("[Stereo] Saliendo...")
            return False

        if key == ord("h"):
            self.show_help = not self.show_help

        elif key == ord("f"):
            self.first_detection = True
            print("[Stereo] Re-deteccion forzada.")

        elif key == ord("c"):
            self.stereo.reset()
            self.left_calibrator = HomographyCalibrator()
            self.ar_viewer = ARViewer(self.left_calibrator)
            self.first_detection = True
            print("[Stereo] Calibracion reiniciada.")

        elif key == ord("v"):
            if self.virtual_scene_3d is None:
                self.virtual_scene_3d = VirtualScene()
                if not self.virtual_scene_3d.initialize(
                    hole_position=self.game_engine.state.hole_position
                ):
                    self.virtual_scene_3d = None
            else:
                self.virtual_scene_3d.stop()
                self.virtual_scene_3d = None

        elif key == ord("n"):
            self.game_engine.restart_game()
            self.virtual_scene_2d.clear_trail()
            self.ar_viewer.clear_trail()
            if self.virtual_scene_3d is not None:
                self.virtual_scene_3d.clear_trail()
                self.virtual_scene_3d.update_hole_position(
                    self.game_engine.state.hole_position
                )

        elif key == ord("r"):
            self.game_engine.restart_current_hole()
            self.virtual_scene_2d.clear_trail()
            self.ar_viewer.clear_trail()

        elif key in (ord("g"), ord("G")):
            self.show_placement_guide = not self.show_placement_guide
            print(
                f"[Stereo] Guia movil: {'ON' if self.show_placement_guide else 'OFF'} "
                f"(camara {'izquierda' if self.guide_on_side == 'left' else 'derecha'})"
            )

        return True

    def _cleanup(self):
        if self.virtual_scene_3d is not None:
            self.virtual_scene_3d.stop()

        if self.cap_left is not None:
            self.cap_left.release()
        if self.cap_right is not None:
            self.cap_right.release()

        cv2.destroyAllWindows()


def main():
    args = parse_args()
    app = StereoMiniGolfApp(args)
    app.run()


if __name__ == "__main__":
    main()
