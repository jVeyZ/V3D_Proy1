"""Simple demo for StereoGreenTracker on two cameras."""

import argparse
import cv2

from ball_tracker import StereoGreenTracker
import game_config as config


def parse_args():
    parser = argparse.ArgumentParser(description="Green ball tracker demo")
    parser.add_argument("--camera-left", type=int, default=2)
    parser.add_argument("--camera-right", type=int, default=1)
    return parser.parse_args()


def main():
    args = parse_args()

    cap_left = cv2.VideoCapture(args.camera_left)
    cap_right = cv2.VideoCapture(args.camera_right)
    if not cap_left.isOpened() or not cap_right.isOpened():
        print(f"[Demo] No se pudo abrir cámaras {args.camera_left}/{args.camera_right}")
        return

    cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
    cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
    cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)

    tracker = StereoGreenTracker()
    first_detection = True

    print("[Demo] Tracking bola verde en estéreo. Pulsa q para salir.")

    while True:
        ok_l, frame_left = cap_left.read()
        ok_r, frame_right = cap_right.read()
        if not ok_l or not ok_r:
            break

        if first_detection:
            det_left, det_right = tracker.detect_both(frame_left, frame_right)
            if det_left is not None and det_right is not None:
                first_detection = False
        else:
            det_left, det_right = tracker.update(frame_left, frame_right)

        tracker.left.draw_trail(frame_left)
        tracker.right.draw_trail(frame_right)
        tracker.left.draw_detection(frame_left, det_left)
        tracker.right.draw_detection(frame_right, det_right)

        cv2.imshow("Ball Tracker Left", frame_left)
        cv2.imshow("Ball Tracker Right", frame_right)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break

    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
