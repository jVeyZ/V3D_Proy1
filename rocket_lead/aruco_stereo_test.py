"""Detección de ArUco en stereo - Basado en PyImageSearch."""

import argparse
import cv2
import numpy as np
import time

import game_config as config


# Diccionario completo de tipos ArUco soportados por OpenCV
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


def parse_args():
    parser = argparse.ArgumentParser(description="Stereo ArUco detection (PyImageSearch style)")
    parser.add_argument("--camera-left", type=int, default=config.CAMERA_LEFT)
    parser.add_argument("--camera-right", type=int, default=config.CAMERA_RIGHT)
    parser.add_argument("-t", "--type", type=str, default="DICT_4X4_50",
                       help="Tipo de ArUco a detectar")
    parser.add_argument("--no-resize", action="store_true",
                       help="No redimensionar frames (para alta resolución)")
    return parser.parse_args()


def detect_aruco(frame, aruco_dict, aruco_params):
    """
    Detecta ArUco markers en un frame.
    Compatible con OpenCV 4.7+ y versiones anteriores.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    try:
        # OpenCV 4.7+
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        corners, ids, rejected = detector.detectMarkers(gray)
    except AttributeError:
        # OpenCV < 4.7
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, aruco_dict, parameters=aruco_params)
    
    return corners, ids, rejected


def draw_aruco(frame, corners, ids):
    """Dibuja bounding boxes, centros e IDs de los markers detectados."""
    if corners is None or len(corners) == 0:
        return frame
    
    # Aplanar IDs
    ids = ids.flatten() if ids is not None else []
    
    for (marker_corner, marker_id) in zip(corners, ids):
        # Extraer esquinas: top-left, top-right, bottom-right, bottom-left
        corners_reshaped = marker_corner.reshape((4, 2))
        (top_left, top_right, bottom_right, bottom_left) = corners_reshaped
        
        # Convertir a enteros
        top_right = (int(top_right[0]), int(top_right[1]))
        bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
        bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
        top_left = (int(top_left[0]), int(top_left[1]))
        
        # Dibujar bounding box
        cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
        cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
        cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
        cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)
        
        # Calcular y dibujar centro
        cX = int((top_left[0] + bottom_right[0]) / 2.0)
        cY = int((top_left[1] + bottom_right[1]) / 2.0)
        cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
        
        # Dibujar ID
        cv2.putText(frame, str(marker_id),
                   (top_left[0], top_left[1] - 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    return frame


def main():
    args = parse_args()
    
    # Verificar que el diccionario existe
    if ARUCO_DICT.get(args.type, None) is None:
        print(f"[ERROR] Tipo de ArUco '{args.type}' no soportado")
        return
    
    print(f"[INFO] OpenCV version: {cv2.__version__}")
    print(f"[INFO] Detectando tags tipo '{args.type}'...")
    
    # Cargar diccionario y parámetros (por defecto, como en PyImageSearch)
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args.type])
    aruco_params = cv2.aruco.DetectorParameters()
    
    # Abrir cámaras (sin CAP_AVFOUNDATION para evitar problemas)
    cap_left = cv2.VideoCapture(args.camera_left)
    cap_right = cv2.VideoCapture(args.camera_right)
    
    if not cap_left.isOpened():
        print(f"[ERROR] No se pudo abrir cámara izquierda ({args.camera_left})")
        return
    if not cap_right.isOpened():
        print(f"[ERROR] No se pudo abrir cámara derecha ({args.camera_right})")
        return
    
    # Configurar resolución
    cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
    cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
    cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    
    # Esperar a que las cámaras se estabilicen
    time.sleep(2.0)
    
    print("[INFO] Pulsa 'q' para salir | 's' para guardar frame | '1-9' cambiar diccionario")
    
    current_type = args.type
    
    while True:
        ok_l, frame_left = cap_left.read()
        ok_r, frame_right = cap_right.read()
        
        if not ok_l or not ok_r:
            print("[ERROR] Fallo en captura de frame")
            break
        
        # Redimensionar si no se desactivó (PyImageSearch recomienda esto para performance)
        if not args.no_resize:
            frame_left = cv2.resize(frame_left, (config.CAMERA_WIDTH, config.CAMERA_HEIGHT))
            frame_right = cv2.resize(frame_right, (config.CAMERA_WIDTH, config.CAMERA_HEIGHT))
        
        # Detectar ArUco en ambas cámaras
        corners_l, ids_l, rejected_l = detect_aruco(frame_left, aruco_dict, aruco_params)
        corners_r, ids_r, rejected_r = detect_aruco(frame_right, aruco_dict, aruco_params)
        
        # Dibujar resultados
        frame_left = draw_aruco(frame_left, corners_l, ids_l)
        frame_right = draw_aruco(frame_right, corners_r, ids_r)
        
        # Info de estado
        ids_l_list = ids_l.flatten().tolist() if ids_l is not None else []
        ids_r_list = ids_r.flatten().tolist() if ids_r is not None else []
        
        missing_l = [i for i in config.FIELD_MARKER_IDS if i not in ids_l_list]
        missing_r = [i for i in config.FIELD_MARKER_IDS if i not in ids_r_list]
        
        status_l = "OK" if not missing_l else f"Faltan: {missing_l}"
        status_r = "OK" if not missing_r else f"Faltan: {missing_r}"
        
        cv2.putText(frame_left, f"[{current_type}] L: {status_l}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame_right, f"[{current_type}] R: {status_r}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Mostrar número de detectados
        cv2.putText(frame_left, f"Detectados: {len(ids_l_list)}", (10, 55),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame_right, f"Detectados: {len(ids_r_list)}", (10, 55),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.imshow("ArUco Left", frame_left)
        cv2.imshow("ArUco Right", frame_right)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord("q") or key == 27:
            break
            
        elif key == ord("s"):
            # Guardar frames para debug
            cv2.imwrite("debug_left.png", frame_left)
            cv2.imwrite("debug_right.png", frame_right)
            print("[INFO] Frames guardados: debug_left.png, debug_right.png")
            
        elif key == ord("1"):
            current_type = "DICT_4X4_50"
        elif key == ord("2"):
            current_type = "DICT_4X4_100"
        elif key == ord("3"):
            current_type = "DICT_5X5_50"
        elif key == ord("4"):
            current_type = "DICT_5X5_100"
        elif key == ord("5"):
            current_type = "DICT_6X6_50"
        elif key == ord("6"):
            current_type = "DICT_6X6_100"
        elif key == ord("7"):
            current_type = "DICT_ARUCO_ORIGINAL"
        elif key == ord("8"):
            current_type = "DICT_APRILTAG_36h11"
        elif key == ord("9"):
            current_type = "DICT_7X7_50"
        
        # Si cambió el diccionario, recargarlo
        if current_type != args.type and key in [ord(str(i)) for i in range(1, 10)]:
            args.type = current_type
            aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args.type])
            print(f"[INFO] Cambiado a diccionario: {args.type}")
    
    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()