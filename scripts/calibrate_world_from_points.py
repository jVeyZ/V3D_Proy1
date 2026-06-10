import argparse
import csv
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.stereo import Stereo3DReconstructor  # noqa: E402


def load_world_points_csv(path):
    required = [
        "world_x_cm",
        "world_y_cm",
        "world_z_cm",
        "left_u",
        "left_v",
        "right_u",
        "right_v",
    ]
    world_points = []
    camera_points = []

    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []
        missing = [name for name in required if name not in fieldnames]
        if missing:
            raise ValueError(f"CSV missing columns: {missing}")

        for row in reader:
            world_points.append(
                [
                    float(row["world_x_cm"]),
                    float(row["world_y_cm"]),
                    float(row["world_z_cm"]),
                ]
            )
            camera_points.append(
                [
                    float(row["left_u"]),
                    float(row["left_v"]),
                    float(row["right_u"]),
                    float(row["right_v"]),
                ]
            )

    return np.asarray(world_points, dtype=np.float64), np.asarray(
        camera_points, dtype=np.float64
    )


def fit_rigid_transform(source_points, target_points):
    """Calcula R,t tal que target ≈ R @ source + t mediante Kabsch."""
    if source_points.shape != target_points.shape or source_points.shape[0] < 3:
        raise ValueError("Need at least 3 source/target 3D point pairs")

    source_centroid = source_points.mean(axis=0)
    target_centroid = target_points.mean(axis=0)
    source_centered = source_points - source_centroid
    target_centered = target_points - target_centroid

    H = source_centered.T @ target_centered
    U, _S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = target_centroid - R @ source_centroid
    transformed = (R @ source_points.T).T + t
    residuals = np.linalg.norm(transformed - target_points, axis=1)
    return R, t, residuals


def write_report(
    path, args, camera_points_3d, world_points, transformed, residuals, R, t
):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("# World Transform Calibration Report\n\n")
        f.write(
            "This file documents the offline transform from left-camera stereo coordinates to the game world frame.\n\n"
        )
        f.write("## Inputs\n\n")
        f.write(f"- Stereo calibration: `{args.stereo_calibration}`\n")
        f.write(f"- World points CSV: `{args.points}`\n")
        f.write(f"- Output transform: `{args.output}`\n")
        f.write(f"- Number of points: {len(world_points)}\n\n")
        f.write("## Convention\n\n")
        f.write("`X_world = R_world_from_left @ X_left_camera + t_world_from_left`\n\n")
        f.write("## Result\n\n")
        f.write(
            f"- RMS world residual: {float(np.sqrt(np.mean(residuals**2))):.4f} cm\n"
        )
        f.write(f"- Mean residual: {float(np.mean(residuals)):.4f} cm\n")
        f.write(f"- Max residual: {float(np.max(residuals)):.4f} cm\n\n")
        f.write("### R_world_from_left\n\n")
        f.write("```\n")
        f.write(np.array2string(R, precision=8, suppress_small=False))
        f.write("\n```\n\n")
        f.write("### t_world_from_left\n\n")
        f.write("```\n")
        f.write(np.array2string(t, precision=8, suppress_small=False))
        f.write("\n```\n\n")
        f.write("## Per-point residuals\n\n")
        f.write(
            "| idx | left-camera X,Y,Z cm | expected world X,Y,Z cm | transformed X,Y,Z cm | residual cm |\n"
        )
        f.write("| --- | --- | --- | --- | --- |\n")
        for idx, (src, dst, pred, res) in enumerate(
            zip(camera_points_3d, world_points, transformed, residuals)
        ):
            f.write(
                f"| {idx} | {src[0]:.2f}, {src[1]:.2f}, {src[2]:.2f} | "
                f"{dst[0]:.2f}, {dst[1]:.2f}, {dst[2]:.2f} | "
                f"{pred[0]:.2f}, {pred[1]:.2f}, {pred[2]:.2f} | {res:.3f} |\n"
            )


def main():
    parser = argparse.ArgumentParser(
        description="Calibrate left-camera stereo coordinates into the Balloon Catch world frame"
    )
    parser.add_argument(
        "--stereo-calibration", default="calibration/stereo_charuco.npz"
    )
    parser.add_argument("--points", default="calibration/world_points.csv")
    parser.add_argument("--output", default="calibration/world_transform.npz")
    parser.add_argument("--report", default="calibration/WORLD_TRANSFORM_REPORT.md")
    args = parser.parse_args()

    world_points, pixel_points = load_world_points_csv(args.points)

    reconstructor = Stereo3DReconstructor()
    if not reconstructor.load_calibration(args.stereo_calibration):
        raise RuntimeError("Could not load stereo calibration")

    camera_points_3d = []
    epipolar_errors = []
    for left_u, left_v, right_u, right_v in pixel_points:
        left_pt = (left_u, left_v)
        right_pt = (right_u, right_v)
        epipolar_errors.append(reconstructor.epipolar_error_px(left_pt, right_pt))
        point_camera = reconstructor.triangulate_camera_left(left_pt, right_pt)
        if point_camera is None:
            raise RuntimeError(
                f"Could not triangulate point pair {left_pt} / {right_pt}"
            )
        camera_points_3d.append(point_camera)
    camera_points_3d = np.asarray(camera_points_3d, dtype=np.float64)

    R, t, residuals = fit_rigid_transform(camera_points_3d, world_points)
    transformed = (R @ camera_points_3d.T).T + t
    world_rms_cm = float(np.sqrt(np.mean(residuals**2)))

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        output_path,
        R_world_from_left=R,
        t_world_from_left=t,
        world_rms_cm=world_rms_cm,
        source_camera_points_cm=camera_points_3d,
        target_world_points_cm=world_points,
        residuals_cm=residuals,
        epipolar_errors_px=np.array(
            [np.nan if e is None else float(e) for e in epipolar_errors],
            dtype=np.float64,
        ),
    )

    write_report(
        Path(args.report),
        args,
        camera_points_3d,
        world_points,
        transformed,
        residuals,
        R,
        t,
    )
    print("Saved world transform:", output_path)
    print("World RMS residual [cm]:", world_rms_cm)
    print("Report:", args.report)


if __name__ == "__main__":
    main()
