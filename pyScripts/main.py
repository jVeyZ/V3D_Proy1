"""
main.py - Aplicación principal de posicionamiento planar con mini-golf.

Integra todos los módulos del proyecto:
  - Calibración de cámara con homografía (calibration.py)
  - Detección manual y automática de la pelota (detection.py)
  - Seguimiento de la pelota en la secuencia (tracking.py)
  - Posicionamiento planar + corrección de altura (positioning.py)
  - Escena virtual / gemelo digital (virtual_scene.py)
  - Visor de realidad aumentada (ar_viewer.py)
  - Motor de juego de mini-golf (game_engine.py)
  - Cámara simulada para pruebas (demo_camera.py)

Uso:
  python main.py              → Modo cámara real (webcam)
  python main.py --demo       → Modo cámara simulada (demo)
  python main.py --demo --auto-calibrate  → Demo con calibración automática
  python main.py --help       → Ayuda

Controles durante la ejecución:
  q / ESC      → Salir
  c            → Recalibrar
  d            → Cambiar modo de detección (manual/automática)
  r            → Reiniciar nivel actual
  n            → Reiniciar partida completa
  h            → Mostrar/ocultar ayuda
  v            → Mostrar/ocultar gemelo digital (Open3D)
  t            → Toggle modo automático (solo demo)
  WASD/flechas → Mover pelota (solo demo)
"""

import sys
import argparse
import cv2
import numpy as np
import time

import config
from calibration import HomographyCalibrator
from detection import ManualDetector, AutomaticDetector
from tracking import create_tracker, OpenCVTracker
from positioning import PlanarPositioner
from virtual_scene import VirtualScene, SimpleVirtualScene
from ar_viewer import ARViewer
from game_engine import MiniGolfEngine
from demo_camera import DemoCamera


def parse_args():
    """Analiza los argumentos de línea de comandos."""
    parser = argparse.ArgumentParser(
        description="Posicionamiento 3D planar con mini-golf virtual"
    )
    parser.add_argument(
        '--demo', action='store_true',
        help='Usar cámara simulada en lugar de webcam real'
    )
    parser.add_argument(
        '--auto-calibrate', action='store_true',
        help='Calibración automática (usa las esquinas conocidas en demo, '
             'o ArUco en cámara real)'
    )
    parser.add_argument(
        '--auto-detect', action='store_true',
        help='Usar detección automática en lugar de manual'
    )
    parser.add_argument(
        '--no-3d', action='store_true',
        help='Desactivar escena virtual 3D (Open3D)'
    )
    parser.add_argument(
        '--camera', type=str, default=str(config.CAMERA_SOURCE),
        help=(f'Índice de la cámara o URL de stream (http/rtsp). '
              f'Por defecto {config.CAMERA_INDEX}')
    )
    parser.add_argument(
        '--tracker', choices=['color', 'csrt', 'kcf', 'mosse'],
        default=config.TRACKING_METHOD,
        help=f'Método de seguimiento (por defecto {config.TRACKING_METHOD})'
    )
    parser.add_argument(
        '--correct-height', action='store_true',
        help='Aplicar corrección de altura (apartado 1.4)'
    )
    return parser.parse_args()


class MiniGolfApp:
    """
    Aplicación principal que orquesta todos los módulos.

    Pipeline por frame:
      1. Capturar imagen de la cámara
      2. Si primer frame: detectar pelota (manual o automática)
      3. Frames siguientes: seguir pelota con tracker
      4. Convertir posición imagen → mundo (homografía + corrección altura)
      5. Actualizar motor de juego
      6. Actualizar escena virtual (gemelo digital)
      7. Dibujar AR sobre la imagen
      8. Mostrar las ventanas
    """

    def __init__(self, args):
        self.args = args
        self.use_demo = args.demo
        self.auto_calibrate = args.auto_calibrate
        self.auto_detect = args.auto_detect
        self.correct_height = args.correct_height

        # Componentes
        self.camera = None
        self.calibrator = HomographyCalibrator()
        self.manual_detector = ManualDetector()
        self.auto_detector = AutomaticDetector()
        self.tracker = create_tracker(args.tracker)
        self.positioner = None
        self.virtual_scene_3d = None
        self.virtual_scene_2d = SimpleVirtualScene()
        self.ar_viewer = None
        self.game_engine = MiniGolfEngine()

        # Estado
        self._first_frame = True
        self._show_help = False
        self._use_3d = not args.no_3d
        self._detection_mode = 'auto' if args.auto_detect else 'manual'
        self._running = True

        # Indicador para evitar spam de mensaje en modo manual
        self._manual_hint_shown = False

    def run(self):
        """Bucle principal de la aplicación."""
        print("\n" + "=" * 60)
        print("   MINI-GOLF VIRTUAL - Posicionamiento Planar")
        print("=" * 60)

        # Paso 1: Inicializar cámara
        if not self._init_camera():
            return

        # Paso 2: Calibración
        if not self._calibrate():
            self.camera.release()
            return

        # Paso 3: Inicializar componentes post-calibración
        self._init_components()

        # Paso 4: Iniciar juego
        self.game_engine.start_game()
        if self._use_3d:
            self._init_3d_scene()

        # Paso 5: Bucle principal
        print("\n[Main] Iniciando bucle principal. Pulsa 'h' para ayuda.\n")
        self._main_loop()

        # Limpieza
        self._cleanup()

    def _init_camera(self):
        """Inicializa la cámara (real o simulada)."""
        if self.use_demo:
            print("[Main] Usando cámara simulada (modo demo)")
            self.camera = DemoCamera()
            return True
        else:
            cam_arg = self.args.camera
            # Intentar convertir a entero si es posible (índice de webcam)
            try:
                cam_src = int(cam_arg)
            except Exception:
                cam_src = cam_arg  # puede ser URL 'http://...'

            print(f"[Main] Intentando abrir cámara {cam_src}...")
            self.camera = cv2.VideoCapture(cam_src)
            if not self.camera.isOpened():
                print("[Main] ERROR: No se pudo abrir la cámara.")
                print(
                    "[Main] Sugerencia: usa --demo para modo simulado o verifica la URL/índice de la cámara.")
                return False
            # Intentar configurar tamaño; no todos los streams permiten esto
            try:
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT,
                                config.CAMERA_HEIGHT)
            except Exception:
                pass
            print(f"[Main] Cámara abierta: "
                  f"{int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
                  f"{int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
            return True

    def _calibrate(self):
        """Realiza la calibración de la homografía."""
        if self.auto_calibrate and self.use_demo:
            ret, frame = self.camera.read()
            if not ret:
                print("[Main] ERROR: No se pudo leer un frame de la cámara.")
                return False
            # En modo demo, usar las esquinas conocidas directamente
            print("[Main] Calibración automática con esquinas conocidas (demo)")
            demo_corners = self.camera.get_calibration_corners()
            self.calibrator.image_corners = demo_corners
            self.calibrator.world_corners = config.WORLD_CORNERS.copy()
            success = self.calibrator._compute_homography()
        elif self.auto_calibrate:
            # En cámara real, intentar ArUco de forma persistente
            print("[Main] Intentando calibración automática con ArUco...")
            print("[Main] Esperando 4 marcadores ArUco (sin fallback a manual).")
            attempts = 0
            success = False
            while True:
                ret, frame = self.camera.read()
                if not ret:
                    print("[Main] ERROR: No se pudo leer un frame de la cámara.")
                    return False

                success = self.calibrator.calibrate_aruco(frame, quiet=True)
                if success:
                    break

                attempts += 1
                if attempts % 30 == 0:
                    print("[Main] Aún no hay 4 ArUcos válidos. Reintentando...")

                key = cv2.waitKey(1) & 0xFF
                if key == 27 or key == ord('q'):
                    print("[Main] Calibración automática cancelada por el usuario.")
                    return False
        else:
            ret, frame = self.camera.read()
            if not ret:
                print("[Main] ERROR: No se pudo leer un frame de la cámara.")
                return False
            # Calibración manual interactiva
            print("[Main] Calibración manual: haz clic en las 4 esquinas")
            success = self.calibrator.calibrate_manual(frame)

        if not success:
            print("[Main] ERROR: La calibración falló.")
            return False

        print("[Main] Calibración exitosa.")
        return True

    def _init_components(self):
        """Inicializa los componentes que dependen de la calibración."""
        self.positioner = PlanarPositioner(self.calibrator)
        self.ar_viewer = ARViewer(self.calibrator)

        if self.correct_height:
            print(f"[Main] Corrección de altura activada: "
                  f"h = {config.BALL_REAL_RADIUS_CM} cm")
            PlanarPositioner.explain_height_correction()

    def _init_3d_scene(self):
        """Inicializa la escena virtual 3D con Open3D."""
        try:
            self.virtual_scene_3d = VirtualScene()
            hole_pos = self.game_engine.state.hole_position
            success = self.virtual_scene_3d.initialize(hole_position=hole_pos)
            if not success:
                print("[Main] No se pudo iniciar Open3D. "
                      "Se usará solo la vista 2D.")
                self.virtual_scene_3d = None
        except Exception as e:
            print(f"[Main] Error al iniciar Open3D: {e}")
            self.virtual_scene_3d = None

    def _main_loop(self):
        """Bucle principal de procesamiento frame a frame."""
        # Configurar callback del ratón para la ventana principal
        cv2.namedWindow("Mini-Golf AR", cv2.WINDOW_NORMAL)
        # Usamos un callback centralizado que delega en la cámara demo (si existe)
        # y captura clicks para detección manual no bloqueante.
        cv2.setMouseCallback("Mini-Golf AR", self._on_mouse_callback)

        # Guardar último frame para uso por la detección manual no bloqueante
        self._last_frame = None
        fps_timer = time.time()
        frame_count = 0
        fps_display = 0
        open3d_retry_counter = 30

        while self._running:
            # 1. Capturar frame
            ret, frame = self.camera.read()
            # Guardar el frame más reciente para la detección manual no bloqueante
            self._last_frame = frame.copy() if ret else None
            if not ret:
                print("[Main] No se pudo leer frame. Finalizando.")
                break

            # 2. Si está habilitada la calibración automática con cámara real,
            #    intentar actualizar la homografía en cada frame. Esto hace que
            #    el campo se "mueva" junto al número ArUco si la cámara se
            #    desplaza, evitando que el área de juego quede fijada en píxeles.
            if self.auto_calibrate and not self.use_demo:
                # calibrar_aruco devuelve False si no había 4 marcadores; en ese
                # caso conservamos la calibración previa.
                self.calibrator.calibrate_aruco(frame, quiet=True)

            # 2b. Reintentar Open3D automáticamente para que aparezca por
            #     defecto sin necesidad de pulsar 'v' si falló al inicio.
            if self._use_3d and self.virtual_scene_3d is None:
                open3d_retry_counter += 1
                if open3d_retry_counter >= 30:
                    self._init_3d_scene()
                    open3d_retry_counter = 0

            # 2. Detectar / Seguir pelota
            ball_detection = self._process_detection(frame)

            # 3. Posicionar en el plano
            ball_world_pos = None
            if ball_detection is not None:
                cx, cy, radius = ball_detection
                if self.correct_height:
                    ball_world_pos = self.positioner.get_ball_world_position(
                        (cx, cy), ball_radius_cm=config.BALL_REAL_RADIUS_CM
                    )
                else:
                    ball_world_pos = self.positioner.image_to_plane((cx, cy))

            # 4. Actualizar juego
            game_state = self.game_engine.update(ball_world_pos)

            # 5. Actualizar escenas virtuales
            if ball_world_pos is not None:
                self.virtual_scene_2d.update_ball(ball_world_pos)
                if self.virtual_scene_3d is not None:
                    self.virtual_scene_3d.update_ball_position(ball_world_pos)

                # Trayectoria AR
                img_pos = self.calibrator.world_to_image(ball_world_pos)
                if img_pos is not None:
                    self.ar_viewer.add_trail_point(img_pos)

            # Actualizar hoyo en escenas
            hole_pos = self.game_engine.state.hole_position
            self.virtual_scene_2d.update_hole(hole_pos)

            # Mantener el visualizador 3D actualizado (si está activo)
            if self.virtual_scene_3d is not None:
                # Llamar a step() desde el hilo principal para evitar problemas
                # en plataformas que requieren que la GUI corra en el hilo principal
                self.virtual_scene_3d.step()

            # 6. Renderizar AR
            ar_frame = self.ar_viewer.draw(
                frame,
                game_state=self.game_engine.get_game_state_dict(),
                ball_detection=ball_detection
            )

            # 7. Calcular FPS
            frame_count += 1
            elapsed = time.time() - fps_timer
            if elapsed > 1.0:
                fps_display = frame_count / elapsed
                frame_count = 0
                fps_timer = time.time()

            cv2.putText(ar_frame, f"FPS: {fps_display:.0f}",
                        (10, ar_frame.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            # 8. Mostrar ayuda si está activada
            if self._show_help:
                self._draw_help(ar_frame)

            # 9. Mostrar ventanas
            cv2.imshow("Mini-Golf AR", ar_frame)

            # Vista 2D del gemelo digital
            view_2d = self.virtual_scene_2d.render()
            cv2.imshow("Gemelo Digital 2D", view_2d)

            # 10. Procesar teclado
            key = cv2.waitKey(1) & 0xFF
            if not self._handle_key(key, frame):
                break

        cv2.destroyAllWindows()

    def _process_detection(self, frame):
        """
        Procesa la detección/seguimiento de la pelota.

        - Primer frame: detección (manual o automática)
        - Frames siguientes: seguimiento
        """
        if self._first_frame:
            # Primera detección
            detection = None

            if self._detection_mode == 'auto':
                detection = self.auto_detector.detect(frame)
                if detection is None:
                    print("[Main] Detección automática falló.")
            else:
                # No lanzar el popup de detección manual automáticamente.
                # El usuario debe pulsar 'm' o hacer click en la ventana para iniciar
                # la detección manual. Mostrar la instrucción una sola vez.
                if not self._manual_hint_shown:
                    print(
                        "[Main] Modo MANUAL activo. Pulse 'm' o haga click en la ventana para detectar la pelota.")
                    self._manual_hint_shown = True

            if detection is not None:
                cx, cy, radius = detection
                print(f"[Main] Pelota detectada en: "
                      f"px=({cx}, {cy}), r={radius}")

                # Inicializar tracker
                if hasattr(self.tracker, 'initialize'):
                    if isinstance(self.tracker, OpenCVTracker):
                        self.tracker.initialize(cx, cy, radius, frame)
                    else:
                        self.tracker.initialize(cx, cy, radius)

                self._first_frame = False
                return detection

            return None
        else:
            # Seguimiento
            result = self.tracker.update(frame)

            if result is None or (hasattr(self.tracker, 'is_lost')
                                  and self.tracker.is_lost):
                # Tracker perdido, intentar re-detección
                print("[Main] Tracker perdido. Re-detectando...")
                detection = self.auto_detector.detect(frame)
                if detection is not None:
                    cx, cy, radius = detection
                    self.tracker = create_tracker(self.args.tracker)
                    if isinstance(self.tracker, OpenCVTracker):
                        self.tracker.initialize(cx, cy, radius, frame)
                    else:
                        self.tracker.initialize(cx, cy, radius)
                    return detection
                return None

            return result

    def _on_mouse_callback(self, event, x, y, flags, param):
        """Callback centralizado para la ventana "Mini-Golf AR".

        - Delegamos en la cámara demo (si existe) para manejo de arrastrar.
        - Si estamos en modo MANUAL y en el primer frame, un click izquierdo
          realizará una detección no bloqueante en la posición pulsada.
        """
        # Delegar a la cámara demo (para arrastrar la pelota en modo demo)
        if self.use_demo and hasattr(self.camera, 'mouse_callback'):
            try:
                self.camera.mouse_callback(event, x, y, flags, param)
            except Exception:
                pass

        # Detección manual no bloqueante con click
        if event == cv2.EVENT_LBUTTONDOWN and self._detection_mode == 'manual' and self._first_frame:
            if self._last_frame is None:
                return
            det = self.manual_detector.detect_point(self._last_frame, x, y)
            if det is not None:
                cx, cy, radius = det
                if isinstance(self.tracker, OpenCVTracker):
                    self.tracker.initialize(cx, cy, radius, self._last_frame)
                else:
                    self.tracker.initialize(cx, cy, radius)
                self._first_frame = False
                print(
                    f"[Main] Pelota detectada (click manual): px=({cx}, {cy}), r={radius}")

    def _handle_key(self, key, frame):
        """
        Procesa las pulsaciones de teclado.

        Returns:
            False si debe salir, True en caso contrario.
        """
        if key == 255:
            return True  # Sin pulsación

        # Salir
        if key == ord('q') or key == 27:
            print("[Main] Saliendo...")
            return False

        # Mover pelota en modo demo
        if self.use_demo:
            self.camera.move_ball_keyboard(key)

        # Recalibrar
        if key == ord('c'):
            print("[Main] Recalibrando...")
            ret, frame = self.camera.read()
            if ret:
                if self.auto_calibrate and self.use_demo:
                    demo_corners = self.camera.get_calibration_corners()
                    self.calibrator.image_corners = demo_corners
                    self.calibrator._compute_homography()
                elif self.auto_calibrate and not self.use_demo:
                    # en cámara real con auto calibración, usar ArUco
                    self.calibrator.calibrate_aruco(frame)
                else:
                    self.calibrator.calibrate_manual(frame)
                self.positioner = PlanarPositioner(self.calibrator)
                self.ar_viewer = ARViewer(self.calibrator)

        # Cambiar modo de detección
        elif key == ord('d'):
            self._detection_mode = ('auto' if self._detection_mode == 'manual'
                                    else 'manual')
            self._first_frame = True
            self.tracker = create_tracker(self.args.tracker)
            self.ar_viewer.clear_trail()
            # Reset hint so the message shows once on entering manual
            self._manual_hint_shown = False
            self.ar_viewer.clear_trail()
            if self.virtual_scene_3d:
                self.virtual_scene_3d.clear_trail()

        # Reiniciar partida
        elif key == ord('n'):
            self.game_engine.restart_game()
            self.virtual_scene_2d.clear_trail()
            self.ar_viewer.clear_trail()
            if self.virtual_scene_3d:
                self.virtual_scene_3d.clear_trail()
                self.virtual_scene_3d.update_hole_position(
                    self.game_engine.state.hole_position
                )

        # Toggle ayuda
        elif key == ord('h'):
            self._show_help = not self._show_help

        # Toggle 3D
        elif key == ord('v'):
            if self.virtual_scene_3d is None and self._use_3d is False:
                self._use_3d = True
                self._init_3d_scene()
            elif self.virtual_scene_3d is not None:
                self.virtual_scene_3d.stop()
                self.virtual_scene_3d = None
                self._use_3d = False
                print("[Main] Escena 3D desactivada")

        # Forzar re-detección
        elif key == ord('f'):
            self._first_frame = True
            self.tracker = create_tracker(self.args.tracker)
            print("[Main] Forzando re-detección en el siguiente frame.")

        # Forzar detección manual inmediata (popup)
        elif key == ord('m'):
            print("[Main] Forzando detección manual (popup)...")
            # Si se quiere el popup bloqueante explícitamente, se puede usar
            # manual_detector.detect; de lo contrario recomendamos click en la
            # ventana ('Modo MANUAL').
            detection = None
            if self._last_frame is not None:
                # Hacer una detección no bloqueante en el centro de la imagen
                h, w = self._last_frame.shape[:2]
                detection = self.manual_detector.detect_point(
                    self._last_frame, w // 2, h // 2)

            if detection is None:
                # Fallback: abrir popup si realmente se necesita
                detection = self.manual_detector.detect(self._last_frame)

            if detection is not None:
                cx, cy, radius = detection
                if isinstance(self.tracker, OpenCVTracker):
                    self.tracker.initialize(cx, cy, radius, self._last_frame)
                else:
                    self.tracker.initialize(cx, cy, radius)
                self._first_frame = False
                print(
                    f"[Main] Pelota detectada (manual): px=({cx}, {cy}), r={radius}")

        # Explicar corrección de altura
        elif key == ord('e'):
            PlanarPositioner.explain_height_correction()

        return True

    def _draw_help(self, frame):
        """Dibuja el panel de ayuda sobre el frame."""
        h, w = frame.shape[:2]
        overlay = frame.copy()

        # Fondo semitransparente
        panel_w, panel_h = 420, 340
        x0 = (w - panel_w) // 2
        y0 = (h - panel_h) // 2
        cv2.rectangle(overlay, (x0, y0), (x0 + panel_w, y0 + panel_h),
                      (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.8, frame, 0.2, 0, frame)

        # Título
        cv2.putText(frame, "CONTROLES", (x0 + 160, y0 + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        controls = [
            ("q / ESC", "Salir"),
            ("c", "Recalibrar camara"),
            ("d", "Cambiar deteccion (manual/auto)"),
            ("f", "Forzar re-deteccion"),
            ("m", "Detección manual (popup o centro)"),
            ("Click", "Detección manual no bloqueante (click en ventana)"),
            ("r", "Reiniciar nivel"),
            ("n", "Nueva partida"),
            ("v", "Toggle escena 3D"),
            ("e", "Explicar correccion altura"),
            ("h", "Ocultar esta ayuda"),
        ]

        if self.use_demo:
            controls.extend([
                ("WASD", "Mover pelota"),
                ("t", "Toggle modo automatico"),
            ])

        y = y0 + 60
        for key_str, desc in controls:
            cv2.putText(frame, f"{key_str:>12s}  {desc}",
                        (x0 + 20, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (255, 255, 255), 1)
            y += 22

    def _cleanup(self):
        """Limpieza al salir."""
        if self.virtual_scene_3d is not None:
            self.virtual_scene_3d.stop()
        if self.camera is not None:
            self.camera.release()
        cv2.destroyAllWindows()
        print("[Main] Aplicación finalizada.")


def main():
    """Punto de entrada principal."""
    args = parse_args()

    # Si se lanza sin argumentos y no hay cámara, sugerir modo demo
    if not args.demo:
        # Soporta índice numérico o URL.
        try:
            test_cam_src = int(args.camera)
        except Exception:
            test_cam_src = args.camera

        test_cam = cv2.VideoCapture(test_cam_src)
        if not test_cam.isOpened():
            test_cam.release()
            print("[Main] No se detectó cámara. Iniciando en modo demo...")
            print("[Main] (Usa --demo explícitamente para evitar este mensaje)\n")
            args.demo = True
            args.auto_calibrate = True
            args.auto_detect = True
        else:
            test_cam.release()

    app = MiniGolfApp(args)
    app.run()


if __name__ == "__main__":
    main()
