import argparse
from pathlib import Path
import cv2
import numpy as np


def ensure_charuco_available():
    missing = []
    if not hasattr(cv2.aruco, "interpolateCornersCharuco"):
        missing.append("interpolateCornersCharuco")
    if not hasattr(cv2.aruco, "calibrateCameraCharuco"):
        missing.append("calibrateCameraCharuco")
    if missing:
        missing_str = ", ".join(missing)
        raise RuntimeError(
            f"Charuco functions not available (missing: {missing_str}). "
            "Install opencv-contrib-python."
        )


def detect_charuco(gray, board, aruco_dict, aruco_params):
    try:
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        corners, ids, _ = detector.detectMarkers(gray)
    except AttributeError:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is None or len(ids) == 0:
        return None, None

    ret, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if not ret or ch_ids is None or len(ch_ids) < 4:
        return None, None

    return ch_corners, ch_ids


def create_charuco_board(squares_x, squares_y, square_len_cm, marker_len_cm, aruco_dict):
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        return cv2.aruco.CharucoBoard_create(
            squares_x, squares_y,
            square_len_cm, marker_len_cm,
            aruco_dict
        )
    if hasattr(cv2.aruco, "CharucoBoard"):
        return cv2.aruco.CharucoBoard(
            (squares_x, squares_y),
            square_len_cm, marker_len_cm,
            aruco_dict
        )
    raise RuntimeError("CharucoBoard not available. Install opencv-contrib-python.")


def main():
    parser = argparse.ArgumentParser(description="Calibrate stereo with Charuco")
    parser.add_argument("--input", default="calibration/capture")
    parser.add_argument("--output", default="calibration/stereo_charuco.npz")
    parser.add_argument("--squares-x", type=int, default=5)
    parser.add_argument("--squares-y", type=int, default=7)
    parser.add_argument("--square-len-cm", type=float, default=4.0)
    parser.add_argument("--marker-len-cm", type=float, default=3.0)
    args = parser.parse_args()

    left_dir = Path(args.input) / "left"
    right_dir = Path(args.input) / "right"

    left_files = sorted(left_dir.glob("left_*.png"))
    right_files = sorted(right_dir.glob("right_*.png"))

    right_map = {p.stem.split("_")[-1]: p for p in right_files}
    pairs = []
    for lf in left_files:
        key = lf.stem.split("_")[-1]
        rf = right_map.get(key)
        if rf is not None:
            pairs.append((lf, rf))

    if len(pairs) == 0:
        print("No image pairs found")
        return

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    board = create_charuco_board(
        args.squares_x, args.squares_y,
        args.square_len_cm, args.marker_len_cm,
        aruco_dict
    )
    ensure_charuco_available()

    all_corners_l = []
    all_ids_l = []
    all_corners_r = []
    all_ids_r = []

    objpoints = []
    imgpoints_l = []
    imgpoints_r = []

    image_size = None

    for lf, rf in pairs:
        img_l = cv2.imread(str(lf))
        img_r = cv2.imread(str(rf))
        if img_l is None or img_r is None:
            continue

        gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)
        image_size = (gray_l.shape[1], gray_l.shape[0])

        ch_corners_l, ch_ids_l = detect_charuco(gray_l, board, aruco_dict, aruco_params)
        ch_corners_r, ch_ids_r = detect_charuco(gray_r, board, aruco_dict, aruco_params)

        if ch_corners_l is None or ch_corners_r is None:
            continue

        all_corners_l.append(ch_corners_l)
        all_ids_l.append(ch_ids_l)
        all_corners_r.append(ch_corners_r)
        all_ids_r.append(ch_ids_r)

        ids_l = ch_ids_l.flatten().tolist()
        ids_r = ch_ids_r.flatten().tolist()
        common = sorted(set(ids_l) & set(ids_r))
        if len(common) < 6:
            continue

        id_to_idx_l = {cid: i for i, cid in enumerate(ids_l)}
        id_to_idx_r = {cid: i for i, cid in enumerate(ids_r)}

        if hasattr(board, "chessboardCorners"):
            board_corners = board.chessboardCorners
        elif hasattr(board, "getChessboardCorners"):
            board_corners = board.getChessboardCorners()
        else:
            raise RuntimeError("Charuco board corners not available.")

        board_corners = np.asarray(board_corners, dtype=np.float32).reshape(-1, 3)
        obj = board_corners[common, :]
        img_l_pts = np.array([ch_corners_l[id_to_idx_l[c]] for c in common], dtype=np.float32)
        img_r_pts = np.array([ch_corners_r[id_to_idx_r[c]] for c in common], dtype=np.float32)

        objpoints.append(obj)
        imgpoints_l.append(img_l_pts)
        imgpoints_r.append(img_r_pts)

    if image_size is None or len(all_corners_l) < 8:
        print("Not enough valid detections for calibration")
        return

    if len(objpoints) == 0:
        print("Not enough valid stereo pairs with >= 6 common Charuco corners")
        return

    ret_l, K_l, dist_l, rvecs_l, tvecs_l = cv2.aruco.calibrateCameraCharuco(
        all_corners_l, all_ids_l, board, image_size, None, None
    )
    ret_r, K_r, dist_r, rvecs_r, tvecs_r = cv2.aruco.calibrateCameraCharuco(
        all_corners_r, all_ids_r, board, image_size, None, None
    )

    flags = cv2.CALIB_FIX_INTRINSIC
    criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-6)
    ret_st, K_l, dist_l, K_r, dist_r, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpoints_l, imgpoints_r,
        K_l, dist_l, K_r, dist_r,
        image_size, criteria=criteria, flags=flags
    )

    np.savez(
        args.output,
        K_l=K_l, dist_l=dist_l,
        K_r=K_r, dist_r=dist_r,
        R=R, T=T, E=E, F=F,
        squares_x=args.squares_x, squares_y=args.squares_y,
        square_len_cm=args.square_len_cm, marker_len_cm=args.marker_len_cm,
        image_size=image_size,
        rms_left=ret_l, rms_right=ret_r, rms_stereo=ret_st
    )

    print("Saved calibration:", args.output)
    print("RMS left:", ret_l)
    print("RMS right:", ret_r)
    print("RMS stereo:", ret_st)


if __name__ == "__main__":
    main()
