'''0 - Unrecognized gesture, label: Unknown
1 - Closed fist, label: Closed_Fist
2 - Open palm, label: Open_Palm
3 - Pointing up, label: Pointing_Up
4 - Thumbs down, label: Thumb_Down
5 - Thumbs up, label: Thumb_Up
6 - Victory, label: Victory
7 - Love, label: ILoveYou'''

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.components.containers.landmark import NormalizedLandmark
import numpy as np
import cv2
import time
import threading

mp_drawing = mp.tasks.vision.drawing_utils
mp_styles  = mp.tasks.vision.drawing_styles
mp_hands   = mp.tasks.vision.HandLandmarksConnections

cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

start_time = time.monotonic()

BaseOptions = mp.tasks.BaseOptions
GestureRecognizer = mp.tasks.vision.GestureRecognizer
GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
GestureRecognizerResult = mp.tasks.vision.GestureRecognizerResult
VisionRunningMode = mp.tasks.vision.RunningMode

# Shared state between callback thread and main loop
latest_result: GestureRecognizerResult | None = None
result_lock = threading.Lock()

def on_result(result: GestureRecognizerResult, output_image: mp.Image, timestamp_ms: int):
    #mete en la variable global (reconoce más rápido q los frames se dibujan)
    global latest_result
    with result_lock:
        latest_result = result

options = GestureRecognizerOptions(
    base_options=BaseOptions(model_asset_path="v3d/gesture_recognizer.task"),
    running_mode=VisionRunningMode.LIVE_STREAM,
    result_callback=on_result)
recognizer = GestureRecognizer.create_from_options(options)

def draw_result(frame_bgr, result: GestureRecognizerResult):
    if not result or not result.hand_landmarks:
        return frame_bgr

    h, w, _ = frame_bgr.shape
    out = frame_bgr.copy()

    for i, landmarks in enumerate(result.hand_landmarks):
        # Draw skeleton
        mp_drawing.draw_landmarks(
            out,
            landmarks,
            mp_hands.HAND_CONNECTIONS,
            mp_styles.get_default_hand_landmarks_style(),
            mp_styles.get_default_hand_connections_style())

        # Gesture label above the hand
        if result.gestures and i < len(result.gestures):
            gesture = result.gestures[i][0]
            label = f"{gesture.category_name}  {gesture.score:.0%}"
            xs = [lm.x for lm in landmarks]
            ys = [lm.y for lm in landmarks]
            tx = max(0, int(min(xs) * w))
            ty = max(20, int(min(ys) * h) - 15)
            cv2.putText(out, label, (tx, ty),
                        cv2.FONT_HERSHEY_DUPLEX, 1.0,
                        (88, 205, 54), 2, cv2.LINE_AA)
    return out

while True:
    ret, frame = cam.read()
    if not ret:
        print("Failed to grab frame")
        break

    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    timestamp_ms = int((time.monotonic() - start_time) * 1000)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)

    #Aquí se llama al callback on_result que actualiza latest_result 
    recognizer.recognize_async(mp_image, timestamp_ms)

    with result_lock:
        result_snap = latest_result

    display = draw_result(frame, result_snap)
    cv2.imshow("Gesture Tracker", display)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC key
        break       

cam.release()
cv2.destroyAllWindows()