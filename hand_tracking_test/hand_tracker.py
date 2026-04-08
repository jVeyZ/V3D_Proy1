# STEP 1: Import the necessary modules.
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import numpy as np
import cv2
import open3d as o3d
import time

mp_hands = mp.tasks.vision.HandLandmarksConnections
mp_drawing = mp.tasks.vision.drawing_utils
mp_drawing_styles = mp.tasks.vision.drawing_styles

MARGIN = 10  # pixels
FONT_SIZE = 1
FONT_THICKNESS = 1
HANDEDNESS_TEXT_COLOR = (88, 205, 54) # vibrant green

cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)   # request higher res for better far-field
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

start_time = time.monotonic()


def draw_landmarks_on_image(rgb_image, detection_result):
  hand_landmarks_list = detection_result.hand_landmarks
  handedness_list = detection_result.handedness
  annotated_image = np.copy(rgb_image)

  # Loop through the detected hands to visualize.
  for idx in range(len(hand_landmarks_list)):
    hand_landmarks = hand_landmarks_list[idx]
    handedness = handedness_list[idx]

    # Draw the hand landmarks.
    mp_drawing.draw_landmarks(
      annotated_image,
      hand_landmarks,
      mp_hands.HAND_CONNECTIONS,
      mp_drawing_styles.get_default_hand_landmarks_style(),
      mp_drawing_styles.get_default_hand_connections_style())

    # Get the top left corner of the detected hand's bounding box.
    height, width, _ = annotated_image.shape
    x_coordinates = [landmark.x for landmark in hand_landmarks]
    y_coordinates = [landmark.y for landmark in hand_landmarks]
    text_x = int(min(x_coordinates) * width)
    text_y = int(min(y_coordinates) * height) - MARGIN

    # Draw handedness (left or right hand) on the image.
    cv2.putText(annotated_image, f"{handedness[0].category_name}",
                (text_x, text_y), cv2.FONT_HERSHEY_DUPLEX,
                FONT_SIZE, HANDEDNESS_TEXT_COLOR, FONT_THICKNESS, cv2.LINE_AA)

  return annotated_image

# STEP 2: Create an HandLandmarker object.
base_options = python.BaseOptions(model_asset_path='hand_landmarker.task')
options = vision.HandLandmarkerOptions(base_options=base_options,
                                        running_mode=vision.RunningMode.VIDEO,
                                        num_hands=1,
                                        min_hand_detection_confidence= 0.5,
                                        min_hand_presence_confidence= 0.5,
                                        min_tracking_confidence = 0.5,)
detector = vision.HandLandmarker.create_from_options(options)

# STEP 3: Load the input image.
#image = mp.Image.create_from_file("v3d/hands.jpg")
while True:
    ret, frame = cam.read()
    if not ret:
        print("Failed to grab frame")
        break
    # STEP 4: Detect hand landmarks from the input image.

    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    timestamp_ms = int((time.monotonic() - start_time) * 1000)

    mp_image = mp.Image(mp.ImageFormat.SRGB, rgb_frame)
    detection_result = detector.detect_for_video(mp_image, timestamp_ms)

    # STEP 5: Process the classification result. In this case, visualize it.
    annotated_image = draw_landmarks_on_image(rgb_frame, detection_result)
    cv2.imshow("Hand Landmarks",cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))


    if cv2.waitKey(1) & 0xFF == 27:  # ESC key
        break    

cam.release()
cv2.destroyAllWindows()