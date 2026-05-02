"""
main_penaltis.py - Une PyBullet + cámara + tracker + calibración
"""
import threading
import queue
import time
import numpy as np
import cv2
import pybullet as p

# Tus módulos existentes
import game_config as config
from calibration import HomographyCalibrator
from ball_tracker import GreenBallTracker
from penaltis_coches import (init_simulation, CarController, CameraController, 
                             FIELD_WIDTH, FIELD_LENGTH, create_tracked_ball)

# ─── Colas para comunicación thread-safe ───
ball_queue = queue.Queue(maxsize=3)   # (x_world_cm, y_world_cm)
frame_queue = queue.Queue(maxsize=2)  # frame para debug (opcional)

# ─── HILO DE CÁMARA ───
def camera_worker(camera_src=0):
    cap = cv2.VideoCapture(camera_src)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    
    if not cap.isOpened():
        print("[CAM] ERROR: No se pudo abrir cámara")
        return
    
    tracker = GreenBallTracker()
    calibrator = HomographyCalibrator()
    
    # Calibración con ArUco (o manual si falla)
    print("[CAM] Buscando ArUcos para calibrar...")
    calibrated = False
    while not calibrated:
        ret, frame = cap.read()
        if not ret:
            continue
        calibrated = calibrator.calibrate_aruco(frame, quiet=True)
        if not calibrated:
            # Si no hay ArUcos, haz calibración manual una vez
            print("[CAM] ArUco no detectado. Haz clic en 4 esquinas...")
            calibrated = calibrator.calibrate_manual(frame)
    
    print("[CAM] Calibración OK. Tracking activo.")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        
        det = tracker.update(frame)
        
        if det is not None:
            cx, cy, radius = det
            world_pos = calibrator.image_to_world((cx, cy))
            if world_pos is not None:
                try:
                    ball_queue.put_nowait(world_pos)
                except queue.Full:
                    pass  # descartar si el main no lee tan rápido
        
        # Debug visual (opcional, puedes quitarlo)
        tracker.draw_detection(frame, det)
        tracker.draw_trail(frame)
        try:
            frame_queue.put_nowait(frame)
        except queue.Full:
            pass
        
        time.sleep(0.001)


# ─── HILO PRINCIPAL (PYBULLET) ───
def main():
    print("=" * 50)
    print("  PENALTIS COCHES + TRACKING")
    print("=" * 50)
    
    # 1. Iniciar PyBullet
    physicsClient, car_id, goal_ids = init_simulation()
    car = CarController(car_id)
    cam = CameraController()
    ball_id = create_tracked_ball()
    
    # 2. Lanzar cámara en segundo plano
    cam_thread = threading.Thread(target=camera_worker, args=(config.CAMERA_INDEX,), daemon=True)
    cam_thread.start()
    
    running = True
    last_time = time.time()
    
    while running:
        now = time.time()
        dt = now - last_time
        last_time = now
        
        # --- Input PyBullet ---
        keys = p.getKeyboardEvents()
        if ord('q') in keys or 27 in keys:
            running = False
            continue
        if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
            p.resetBasePositionAndOrientation(car_id, [0, -4, 0.5], [0,0,0,1])
            car.speed = 0
            car.heading = 0
        if ord('c') in keys and keys[ord('c')] & p.KEY_WAS_TRIGGERED:
            cam.next_mode()
        
        # --- Update coche ---
        car_pos = car.update(keys, dt)
        cam.update(car_pos)
        
        # --- Update bola desde cámara ---
        try:
            world_pos = ball_queue.get_nowait()  # [x, y] en cm
            # Mapear área real (60x40 cm) al campo PyBullet
            x_pb = (world_pos[0] - config.PLAY_AREA_WIDTH/2)  * config.SCENE_SCALE
            y_pb = (world_pos[1] - config.PLAY_AREA_HEIGHT/2) * config.SCENE_SCALE
            # Alineamos la portería del juego AR con la portería PyBullet
            y_pb += FIELD_LENGTH/2 - (config.PLAY_AREA_HEIGHT * config.SCENE_SCALE)/2
            
            p.resetBasePositionAndOrientation(
                ball_id, [x_pb, y_pb, 0.08], [0,0,0,1]
            )
            print(f"[BALL] Detección en mundo: ({world_pos[0]:.1f}, {world_pos[1]:.1f}) cm")
        except queue.Empty:
            pass
        
        p.stepSimulation()
        time.sleep(max(0, 0.016 - dt))
    
    p.disconnect()
    print("[GAME] Cerrando...")


if __name__ == "__main__":
    main()