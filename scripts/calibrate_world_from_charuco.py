import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from scripts.calibrate_world_from_points import fit_rigid_transform  # noqa: E402
from src.stereo import Stereo3DReconstructor  # noqa: E402


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
        return np.asarray(board.chessboardCorners, dtype=np.float64).reshape(-1, 3)
    if hasattr(board, "getChessboardCorners"):
        return np.asarray(board.getChessboardCorners(), dtype=np.float64).reshape(-1, 3)
    raise RuntimeError("Charuco board corners not available.")


def detect_charuco(image_bgr, board, aruco_dict, aruco_params):
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    try:
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        marker_corners, marker_ids, _ = detector.detectMarkers(gray)
    except AttributeError:
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            gray, aruco_dict, parameters=aruco_params
        )

    if marker_ids is None or len(marker_ids) == 0:
        return None, None, marker_corners, marker_ids

    ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners, marker_ids, gray, board
    )
    if not ret or charuco_ids is None or len(charuco_ids) < 4:
        return None, None, marker_corners, marker_ids
    return charuco_corners, charuco_ids, marker_corners, marker_ids


def save_annotated(
    path, image_bgr, marker_corners, marker_ids, charuco_corners, charuco_ids
):
    out = image_bgr.copy()
    if marker_ids is not None:
        cv2.aruco.drawDetectedMarkers(out, marker_corners, marker_ids)
    if charuco_corners is not None:
        cv2.aruco.drawDetectedCornersCharuco(out, charuco_corners, charuco_ids)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), out)


def write_report(
    path,
    args,
    common_ids,
    camera_points,
    world_points,
    transformed,
    residuals,
    epipolar_errors,
    R,
    t,
):
    path.parent.mkdir(parents=True, exist_ok=True)
    rms = float(np.sqrt(np.mean(residuals**2)))
    with open(path, "w", encoding="utf-8") as f:
        f.write("# Automatic World Calibration from ChArUco\n\n")
        f.write(
            "This report documents the automatic offline calibration from left-camera stereo coordinates to the Balloon Catch world frame.\n\n"
        )
        f.write("## Inputs\n\n")
        f.write(f"- Stereo calibration: `{args.stereo_calibration}`\n")
        f.write(f"- Left image: `{args.left_image}`\n")
        f.write(f"- Right image: `{args.right_image}`\n")
        f.write(f"- Output transform: `{args.output}`\n")
        f.write(f"- Common ChArUco corners: {len(common_ids)}\n")
        f.write(
            f"- Board world offset: ({args.world_offset_x_cm}, {args.world_offset_y_cm}, {args.world_offset_z_cm}) cm\n\n"
        )
        f.write("## Result\n\n")
        f.write(f"- RMS world residual: {rms:.4f} cm\n")
        f.write(f"- Mean residual: {float(np.mean(residuals)):.4f} cm\n")
        f.write(f"- Max residual: {float(np.max(residuals)):.4f} cm\n")
        finite_epi = epipolar_errors[np.isfinite(epipolar_errors)]
        if finite_epi.size:
            f.write(f"- Mean epipolar error: {float(np.mean(finite_epi)):.4f} px\n")
            f.write(f"- Max epipolar error: {float(np.max(finite_epi)):.4f} px\n")
        f.write("\n## Convention\n\n")
        f.write("`X_world = R_world_from_left @ X_left_camera + t_world_from_left`\n\n")
        f.write("## R_world_from_left\n\n```\n")
        f.write(np.array2string(R, precision=8, suppress_small=False))
        f.write("\n```\n\n")
        f.write("## t_world_from_left\n\n```\n")
        f.write(np.array2string(t, precision=8, suppress_small=False))
        f.write("\n```\n\n")
        f.write("## Per-corner residuals\n\n")
        f.write(
            "| id | camera-left XYZ cm | world target XYZ cm | transformed XYZ cm | residual cm | epipolar px |\n"
        )
        f.write("| --- | --- | --- | --- | --- | --- |\n")
        for cid, src, dst, pred, res, epi in zip(
            common_ids,
            camera_points,
            world_points,
            transformed,
            residuals,
            epipolar_errors,
        ):
            epi_txt = "n/a" if not np.isfinite(epi) else f"{epi:.3f}"
            f.write(
                f"| {int(cid)} | {src[0]:.2f}, {src[1]:.2f}, {src[2]:.2f} | "
                f"{dst[0]:.2f}, {dst[1]:.2f}, {dst[2]:.2f} | "
                f"{pred[0]:.2f}, {pred[1]:.2f}, {pred[2]:.2f} | {res:.3f} | {epi_txt} |\n"
            )


def main():
    parser = argparse.ArgumentParser(
        description="Automatically create world_transform.npz from one stereo ChArUco image pair"
    )
    parser.add_argument(
        "--stereo-calibration", default="calibration/stereo_charuco.npz"
    )
    parser.add_argument("--left-image", required=True)
    parser.add_argument("--right-image", required=True)
    parser.add_argument("--output", default="calibration/world_transform.npz")
    parser.add_argument("--report", default="calibration/WORLD_TRANSFORM_REPORT.md")
    parser.add_argument("--annotated-dir", default="calibration/world_report/images")
    parser.add_argument("--squares-x", type=int, default=5)
    parser.add_argument("--squares-y", type=int, default=7)
    parser.add_argument("--square-len-cm", type=float, default=4.0)
    parser.add_argument("--marker-len-cm", type=float, default=3.0)
    parser.add_argument("--world-offset-x-cm", type=float, default=0.0)
    parser.add_argument("--world-offset-y-cm", type=float, default=0.0)
    parser.add_argument("--world-offset-z-cm", type=float, default=0.0)
    parser.add_argument("--min-corners", type=int, default=8)
    args = parser.parse_args()

    img_l = cv2.imread(args.left_image)
    img_r = cv2.imread(args.right_image)
    if img_l is None:
        raise RuntimeError(f"Could not read left image: {args.left_image}")
    if img_r is None:
        raise RuntimeError(f"Could not read right image: {args.right_image}")

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    board = create_charuco_board(
        args.squares_x,
        args.squares_y,
        args.square_len_cm,
        args.marker_len_cm,
        aruco_dict,
    )
    board_corners = get_board_corners(board)

    ch_l, ids_l, marker_corners_l, marker_ids_l = detect_charuco(
        img_l, board, aruco_dict, aruco_params
    )
    ch_r, ids_r, marker_corners_r, marker_ids_r = detect_charuco(
        img_r, board, aruco_dict, aruco_params
    )
    if ch_l is None or ch_r is None:
        raise RuntimeError("Could not detect ChArUco corners in both images")

    save_annotated(
        Path(args.annotated_dir) / "world_left_detected.png",
        img_l,
        marker_corners_l,
        marker_ids_l,
        ch_l,
        ids_l,
    )
    save_annotated(
        Path(args.annotated_dir) / "world_right_detected.png",
        img_r,
        marker_corners_r,
        marker_ids_r,
        ch_r,
        ids_r,
    )

    ids_l_flat = ids_l.flatten().astype(int).tolist()
    ids_r_flat = ids_r.flatten().astype(int).tolist()
    common_ids = sorted(set(ids_l_flat) & set(ids_r_flat))
    if len(common_ids) < args.min_corners:
        raise RuntimeError(
            f"Only {len(common_ids)} common ChArUco corners found. Need at least {args.min_corners}."
        )

    idx_l = {cid: i for i, cid in enumerate(ids_l_flat)}
    idx_r = {cid: i for i, cid in enumerate(ids_r_flat)}

    reconstructor = Stereo3DReconstructor()
    if not reconstructor.load_calibration(args.stereo_calibration):
        raise RuntimeError("Could not load stereo calibration")

    offset = np.array(
        [args.world_offset_x_cm, args.world_offset_y_cm, args.world_offset_z_cm],
        dtype=np.float64,
    )

    camera_points = []
    world_points = []
    used_ids = []
    epipolar_errors = []
    for cid in common_ids:
        left_pt = ch_l[idx_l[cid], 0, :]
        right_pt = ch_r[idx_r[cid], 0, :]
        camera_point = reconstructor.triangulate_camera_left(left_pt, right_pt)
        if camera_point is None:
            continue
        camera_points.append(camera_point)
        world_points.append(board_corners[cid] + offset)
        used_ids.append(cid)
        epi = reconstructor.epipolar_error_px(left_pt, right_pt)
        epipolar_errors.append(np.nan if epi is None else float(epi))

    camera_points = np.asarray(camera_points, dtype=np.float64)
    world_points = np.asarray(world_points, dtype=np.float64)
    epipolar_errors = np.asarray(epipolar_errors, dtype=np.float64)
    if len(camera_points) < args.min_corners:
        raise RuntimeError("Not enough triangulated ChArUco corners after filtering")

    R, t, residuals = fit_rigid_transform(camera_points, world_points)
    transformed = (R @ camera_points.T).T + t
    world_rms_cm = float(np.sqrt(np.mean(residuals**2)))

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        output_path,
        R_world_from_left=R,
        t_world_from_left=t,
        world_rms_cm=world_rms_cm,
        source_camera_points_cm=camera_points,
        target_world_points_cm=world_points,
        residuals_cm=residuals,
        charuco_corner_ids=np.asarray(used_ids, dtype=np.int32),
        epipolar_errors_px=epipolar_errors,
        world_offset_cm=offset,
    )

    write_report(
        Path(args.report),
        args,
        np.asarray(used_ids, dtype=np.int32),
        camera_points,
        world_points,
        transformed,
        residuals,
        epipolar_errors,
        R,
        t,
    )

    print("Saved world transform:", output_path)
    print("World RMS residual [cm]:", world_rms_cm)
    print("Report:", args.report)
    print("Annotated detections:", args.annotated_dir)


if __name__ == "__main__":
    main()
