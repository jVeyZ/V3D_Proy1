import argparse
from pathlib import Path

import cv2


def interpolate_charuco(corners, ids, gray, board):
    if corners is None or ids is None:
        return None
    if hasattr(cv2.aruco, "interpolateCornersCharuco"):
        return cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    return None


def detect_markers(detector, gray, aruco_dict, aruco_params):
    if detector is None or detector == "legacy":
        return cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    return detector.detectMarkers(gray)


def create_charuco_board(
    squares_x, squares_y, square_len_cm, marker_len_cm, aruco_dict
):
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        return cv2.aruco.CharucoBoard_create(
            squares_x, squares_y, square_len_cm, marker_len_cm, aruco_dict
        )
    if hasattr(cv2.aruco, "CharucoBoard"):
        return cv2.aruco.CharucoBoard(
            (squares_x, squares_y), square_len_cm, marker_len_cm, aruco_dict
        )
    raise RuntimeError("CharucoBoard not available. Install opencv-contrib-python.")


def parse_camera_source(value):
    """Acepta índice local ('0') o URL de stream ('http://.../video')."""
    text = str(value)
    return int(text) if text.isdigit() else text


def main():
    parser = argparse.ArgumentParser(description="Capture stereo Charuco images")
    parser.add_argument("--left", default="0", help="Índice de cámara o URL HTTP/RTSP")
    parser.add_argument("--right", default="1", help="Índice de cámara o URL HTTP/RTSP")
    parser.add_argument("--output", default="calibration/capture")
    parser.add_argument("--squares-x", type=int, default=5)
    parser.add_argument("--squares-y", type=int, default=7)
    parser.add_argument("--square-len-cm", type=float, default=4.0)
    parser.add_argument("--marker-len-cm", type=float, default=3.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    out_dir = Path(args.output)
    left_dir = out_dir / "left"
    right_dir = out_dir / "right"
    left_dir.mkdir(parents=True, exist_ok=True)
    right_dir.mkdir(parents=True, exist_ok=True)

    existing = sorted(left_dir.glob("left_*.png"))
    idx = len(existing)

    left_source = parse_camera_source(args.left)
    right_source = parse_camera_source(args.right)
    print(f"Left source: {left_source}")
    print(f"Right source: {right_source}")

    cap_l = cv2.VideoCapture(left_source)
    cap_r = cv2.VideoCapture(right_source)
    if not cap_l.isOpened() or not cap_r.isOpened():
        print("ERROR: Could not open cameras")
        return

    cap_l.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap_l.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap_r.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap_r.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    actual_w_l = int(cap_l.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h_l = int(cap_l.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_w_r = int(cap_r.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h_r = int(cap_r.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Requested res: {args.width}x{args.height}")
    print(f"Left res: {actual_w_l}x{actual_h_l} | Right res: {actual_w_r}x{actual_h_r}")

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    try:
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    except AttributeError:
        detector = "legacy"

    board = create_charuco_board(
        args.squares_x,
        args.squares_y,
        args.square_len_cm,
        args.marker_len_cm,
        aruco_dict,
    )

    print("Press 's' to save a pair, 'q' to quit.")
    resolution_warned = False
    while True:
        ok_l, frame_l = cap_l.read()
        ok_r, frame_r = cap_r.read()
        if not ok_l or not ok_r:
            continue

        h_l, w_l = frame_l.shape[:2]
        h_r, w_r = frame_r.shape[:2]
        if (w_l, h_l) != (w_r, h_r):
            if not resolution_warned:
                print(f"WARNING: resolution mismatch - Left: {w_l}x{h_l}, Right: {w_r}x{h_r}. "
                      f"Normalizing both to {min(w_l, w_r)}x{min(h_l, h_r)}.")
                resolution_warned = True
            target_w = min(w_l, w_r)
            target_h = min(h_l, h_r)
            if (w_l, h_l) != (target_w, target_h):
                frame_l = cv2.resize(frame_l, (target_w, target_h))
            if (w_r, h_r) != (target_w, target_h):
                frame_r = cv2.resize(frame_r, (target_w, target_h))

        frame_l_raw = frame_l
        frame_r_raw = frame_r
        frame_l_vis = frame_l_raw.copy()
        frame_r_vis = frame_r_raw.copy()

        gray_l = cv2.cvtColor(frame_l_raw, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(frame_r_raw, cv2.COLOR_BGR2GRAY)

        corners_l, ids_l, _ = detect_markers(detector, gray_l, aruco_dict, aruco_params)
        corners_r, ids_r, _ = detect_markers(detector, gray_r, aruco_dict, aruco_params)

        left_count = 0
        right_count = 0
        left_ids = []
        right_ids = []

        if ids_l is not None:
            cv2.aruco.drawDetectedMarkers(frame_l_vis, corners_l, ids_l)
            left_ids = ids_l.flatten().tolist()
            ch = interpolate_charuco(corners_l, ids_l, gray_l, board)
            if ch is not None:
                _, ch_corners, ch_ids = ch
                if ch_corners is not None:
                    cv2.aruco.drawDetectedCornersCharuco(
                        frame_l_vis, ch_corners, ch_ids
                    )
                    left_count = len(ch_ids)

        if ids_r is not None:
            cv2.aruco.drawDetectedMarkers(frame_r_vis, corners_r, ids_r)
            right_ids = ids_r.flatten().tolist()
            ch = interpolate_charuco(corners_r, ids_r, gray_r, board)
            if ch is not None:
                _, ch_corners, ch_ids = ch
                if ch_corners is not None:
                    cv2.aruco.drawDetectedCornersCharuco(
                        frame_r_vis, ch_corners, ch_ids
                    )
                    right_count = len(ch_ids)

        left_text = f"IDs: {left_ids[:8]}" if left_ids else "IDs: none"
        left_status = f"L corners: {left_count}"
        cv2.putText(
            frame_l_vis,
            left_status,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0) if left_count >= 4 else (0, 165, 255),
            2,
        )
        cv2.putText(
            frame_l_vis,
            left_text,
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
        )

        right_status = f"R corners: {right_count}"
        cv2.putText(
            frame_r_vis,
            right_status,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0) if right_count >= 4 else (0, 165, 255),
            2,
        )

        cv2.imshow("Charuco Left", frame_l_vis)
        cv2.imshow("Charuco Right", frame_r_vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            left_path = left_dir / f"left_{idx:03d}.png"
            right_path = right_dir / f"right_{idx:03d}.png"
            cv2.imwrite(str(left_path), frame_l_raw)
            cv2.imwrite(str(right_path), frame_r_raw)
            print(f"Saved pair {idx}")
            idx += 1
        elif key == ord("q") or key == 27:
            break

    cap_l.release()
    cap_r.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
