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


def detect_markers(gray, aruco_dict, aruco_params):
    try:
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        return detector.detectMarkers(gray)
    except AttributeError:
        return cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)


def detect_charuco(gray, board, aruco_dict, aruco_params):
    corners, ids, rejected = detect_markers(gray, aruco_dict, aruco_params)
    if ids is None or len(ids) == 0:
        return None, None, corners, ids, rejected

    ret, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, board
    )
    if not ret or ch_ids is None or len(ch_ids) < 4:
        return None, None, corners, ids, rejected

    return ch_corners, ch_ids, corners, ids, rejected


def create_charuco_board(
    squares_x, squares_y, square_len_cm, marker_len_cm, aruco_dict
):
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        return cv2.aruco.CharucoBoard_create(
            squares_x,
            squares_y,
            square_len_cm,
            marker_len_cm,
            aruco_dict,
        )
    if hasattr(cv2.aruco, "CharucoBoard"):
        return cv2.aruco.CharucoBoard(
            (squares_x, squares_y),
            square_len_cm,
            marker_len_cm,
            aruco_dict,
        )
    raise RuntimeError("CharucoBoard not available. Install opencv-contrib-python.")


def get_board_corners(board):
    if hasattr(board, "chessboardCorners"):
        return np.asarray(board.chessboardCorners, dtype=np.float32).reshape(-1, 3)
    if hasattr(board, "getChessboardCorners"):
        return np.asarray(board.getChessboardCorners(), dtype=np.float32).reshape(-1, 3)
    raise RuntimeError("Charuco board corners not available.")


def save_annotated_pair(report_images_dir, idx, img_l, img_r, data_l, data_r):
    report_images_dir.mkdir(parents=True, exist_ok=True)
    ch_corners_l, ch_ids_l, marker_corners_l, marker_ids_l = data_l
    ch_corners_r, ch_ids_r, marker_corners_r, marker_ids_r = data_r
    vis_l = img_l.copy()
    vis_r = img_r.copy()
    if marker_ids_l is not None:
        cv2.aruco.drawDetectedMarkers(vis_l, marker_corners_l, marker_ids_l)
    if marker_ids_r is not None:
        cv2.aruco.drawDetectedMarkers(vis_r, marker_corners_r, marker_ids_r)
    if ch_corners_l is not None:
        cv2.aruco.drawDetectedCornersCharuco(vis_l, ch_corners_l, ch_ids_l)
    if ch_corners_r is not None:
        cv2.aruco.drawDetectedCornersCharuco(vis_r, ch_corners_r, ch_ids_r)
    cv2.imwrite(str(report_images_dir / f"left_detected_{idx:03d}.png"), vis_l)
    cv2.imwrite(str(report_images_dir / f"right_detected_{idx:03d}.png"), vis_r)


def write_report(
    report_path,
    args,
    pair_count,
    valid_mono_count,
    valid_stereo_count,
    image_size,
    K_l,
    dist_l,
    K_r,
    dist_r,
    R,
    T,
    ret_l,
    ret_r,
    ret_st,
):
    report_path.parent.mkdir(parents=True, exist_ok=True)
    baseline = np.linalg.norm(T)
    status = "OK" if ret_st <= args.max_acceptable_rms else "REVISAR"
    with open(report_path, "w", encoding="utf-8") as f:
        f.write("# Stereo Calibration Report\n\n")
        f.write("## Summary\n\n")
        f.write(f"- Status: **{status}**\n")
        f.write(f"- Input folder: `{args.input}`\n")
        f.write(f"- Output calibration: `{args.output}`\n")
        f.write(f"- Image size: {image_size}\n")
        f.write(f"- Captured pairs found: {pair_count}\n")
        f.write(f"- Valid mono Charuco detections per camera: {valid_mono_count}\n")
        f.write(f"- Valid stereo pairs with common corners: {valid_stereo_count}\n")
        f.write(f"- RMS left: {ret_l:.6f} px\n")
        f.write(f"- RMS right: {ret_r:.6f} px\n")
        f.write(f"- RMS stereo: {ret_st:.6f} px\n")
        f.write(f"- Baseline norm: {baseline:.6f} cm\n")
        f.write(f"- Recommended maximum RMS: {args.max_acceptable_rms:.2f} px\n\n")
        f.write("## Calibration target\n\n")
        f.write(f"- Charuco squares: {args.squares_x} x {args.squares_y}\n")
        f.write(f"- Square length: {args.square_len_cm} cm\n")
        f.write(f"- Marker length: {args.marker_len_cm} cm\n\n")
        f.write("## Matrices\n\n")
        for name, value in [
            ("K_l", K_l),
            ("dist_l", dist_l.ravel()),
            ("K_r", K_r),
            ("dist_r", dist_r.ravel()),
            ("R_right_from_left", R),
            ("T_right_from_left", T.ravel()),
        ]:
            f.write(
                f"### {name}\n\n```\n{np.array2string(value, precision=8, suppress_small=False)}\n```\n\n"
            )
        f.write("## How to use this report\n\n")
        f.write(
            "Include this file and the images in `calibration/report/images/` in the project memory. If RMS stereo is high, recapture more pairs with the board covering the full field of view.\n"
        )


def main():
    parser = argparse.ArgumentParser(
        description="Calibrate stereo with Charuco and generate a report"
    )
    parser.add_argument("--input", default="calibration/capture")
    parser.add_argument("--output", default="calibration/stereo_charuco.npz")
    parser.add_argument("--report-dir", default="calibration/report")
    parser.add_argument("--squares-x", type=int, default=5)
    parser.add_argument("--squares-y", type=int, default=7)
    parser.add_argument("--square-len-cm", type=float, default=4.0)
    parser.add_argument("--marker-len-cm", type=float, default=3.0)
    parser.add_argument("--max-acceptable-rms", type=float, default=2.0)
    parser.add_argument("--max-report-images", type=int, default=12)
    args = parser.parse_args()

    left_dir = Path(args.input) / "left"
    right_dir = Path(args.input) / "right"
    report_dir = Path(args.report_dir)
    report_images_dir = report_dir / "images"

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
        args.squares_x,
        args.squares_y,
        args.square_len_cm,
        args.marker_len_cm,
        aruco_dict,
    )
    ensure_charuco_available()
    board_corners = get_board_corners(board)

    all_corners_l = []
    all_ids_l = []
    all_corners_r = []
    all_ids_r = []
    objpoints = []
    imgpoints_l = []
    imgpoints_r = []
    image_size = None
    annotated_saved = 0

    for pair_idx, (lf, rf) in enumerate(pairs):
        img_l = cv2.imread(str(lf))
        img_r = cv2.imread(str(rf))
        if img_l is None or img_r is None:
            continue

        gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)
        image_size = (gray_l.shape[1], gray_l.shape[0])

        ch_corners_l, ch_ids_l, marker_corners_l, marker_ids_l, _ = detect_charuco(
            gray_l, board, aruco_dict, aruco_params
        )
        ch_corners_r, ch_ids_r, marker_corners_r, marker_ids_r, _ = detect_charuco(
            gray_r, board, aruco_dict, aruco_params
        )
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
        obj = board_corners[common, :]
        img_l_pts = np.array(
            [ch_corners_l[id_to_idx_l[c]] for c in common], dtype=np.float32
        )
        img_r_pts = np.array(
            [ch_corners_r[id_to_idx_r[c]] for c in common], dtype=np.float32
        )
        objpoints.append(obj)
        imgpoints_l.append(img_l_pts)
        imgpoints_r.append(img_r_pts)

        if annotated_saved < args.max_report_images:
            save_annotated_pair(
                report_images_dir,
                pair_idx,
                img_l,
                img_r,
                (ch_corners_l, ch_ids_l, marker_corners_l, marker_ids_l),
                (ch_corners_r, ch_ids_r, marker_corners_r, marker_ids_r),
            )
            annotated_saved += 1

    if image_size is None or len(all_corners_l) < 8:
        print("Not enough valid detections for calibration")
        return
    if len(objpoints) == 0:
        print("Not enough valid stereo pairs with >= 6 common Charuco corners")
        return

    ret_l, K_l, dist_l, _rvecs_l, _tvecs_l = cv2.aruco.calibrateCameraCharuco(
        all_corners_l, all_ids_l, board, image_size, None, None
    )
    ret_r, K_r, dist_r, _rvecs_r, _tvecs_r = cv2.aruco.calibrateCameraCharuco(
        all_corners_r, all_ids_r, board, image_size, None, None
    )

    flags = cv2.CALIB_FIX_INTRINSIC
    criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-6)
    ret_st, K_l, dist_l, K_r, dist_r, R, T, E, F = cv2.stereoCalibrate(
        objpoints,
        imgpoints_l,
        imgpoints_r,
        K_l,
        dist_l,
        K_r,
        dist_r,
        image_size,
        criteria=criteria,
        flags=flags,
    )

    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        args.output,
        K_l=K_l,
        dist_l=dist_l,
        K_r=K_r,
        dist_r=dist_r,
        R=R,
        T=T,
        E=E,
        F=F,
        squares_x=args.squares_x,
        squares_y=args.squares_y,
        square_len_cm=args.square_len_cm,
        marker_len_cm=args.marker_len_cm,
        image_size=image_size,
        pair_count=len(pairs),
        valid_mono_count=len(all_corners_l),
        valid_stereo_count=len(objpoints),
        rms_left=ret_l,
        rms_right=ret_r,
        rms_stereo=ret_st,
    )

    write_report(
        report_dir / "STEREO_CALIBRATION_REPORT.md",
        args,
        len(pairs),
        len(all_corners_l),
        len(objpoints),
        image_size,
        K_l,
        dist_l,
        K_r,
        dist_r,
        R,
        T,
        ret_l,
        ret_r,
        ret_st,
    )

    print("Saved calibration:", args.output)
    print("Saved report:", report_dir / "STEREO_CALIBRATION_REPORT.md")
    print("RMS left:", ret_l)
    print("RMS right:", ret_r)
    print("RMS stereo:", ret_st)
    if ret_st > args.max_acceptable_rms:
        print(
            "WARNING: RMS stereo is high. Recapture calibration pairs before presenting."
        )


if __name__ == "__main__":
    main()
