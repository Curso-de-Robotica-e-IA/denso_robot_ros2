#!/usr/bin/env python3

"""Compute a Robot1->Robot2 transform from calibration point pairs.

This variant keeps the position-only Kabsch estimate, then composes it with an
orientation correction derived from the 6DOF quaternions stored in the
calibration JSON files.

The result is a transform that still uses the XYZ point cloud alignment, but
also incorporates the calibration rotations.
"""

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


def load_pose(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with path.open("r", encoding="utf-8") as infile:
        data = json.load(infile)

    position = data["position"]
    orientation = data.get("orientation", {})
    quaternion = orientation.get("quaternion")
    if quaternion is None:
        raise KeyError(f"Missing orientation.quaternion in {path}")

    point_xyz = np.array([position["x"], position["y"], position["z"]], dtype=float)
    rotation = quaternion_to_rotation_matrix(
        quaternion["x"], quaternion["y"], quaternion["z"], quaternion["w"]
    )
    return point_xyz, rotation


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    quaternion = np.array([x, y, z, w], dtype=float)
    norm = np.linalg.norm(quaternion)
    if norm == 0.0:
        raise ValueError("Zero-length quaternion is not valid")

    x, y, z, w = quaternion / norm

    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def average_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    if not rotations:
        raise ValueError("At least one rotation matrix is required")

    mean_matrix = np.mean(np.stack(rotations, axis=0), axis=0)
    u_mat, _, v_t_mat = np.linalg.svd(mean_matrix)
    rotation = u_mat @ v_t_mat

    if np.linalg.det(rotation) < 0.0:
        u_mat[:, -1] *= -1.0
        rotation = u_mat @ v_t_mat

    return rotation


def compute_rigid_transform_svd(points_r1: np.ndarray, points_r2: np.ndarray):
    centroid_r1 = points_r1.mean(axis=0)
    centroid_r2 = points_r2.mean(axis=0)

    centered_r1 = points_r1 - centroid_r1
    centered_r2 = points_r2 - centroid_r2

    covariance = centered_r1.T @ centered_r2
    u_mat, singular_values, v_t_mat = np.linalg.svd(covariance)

    rotation = v_t_mat.T @ u_mat.T
    # Enforce proper rotation (det = +1) by correcting reflection if needed.
    if np.linalg.det(rotation) < 0.0:
        v_t_mat[-1, :] *= -1.0
        rotation = v_t_mat.T @ u_mat.T

    translation = centroid_r2 - rotation @ centroid_r1

    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation

    aligned_r1 = (rotation @ points_r1.T).T + translation
    residuals = points_r2 - aligned_r1
    point_errors = np.linalg.norm(residuals, axis=1)
    rmse = float(np.sqrt(np.mean(point_errors**2)))

    return transform, rotation, translation, singular_values, point_errors, rmse


def compose_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Compute Robot1->Robot2 rigid transform from calib_tool_tf-P*.json "
            "point pairs using SVD (Kabsch) and a 6DOF rotation correction."
        )
    )
    parser.add_argument(
        "--points-dir",
        type=Path,
        default=Path(__file__).resolve().parent.parent,
        help="Directory containing calib_tool_tf-P*.json files",
    )
    parser.add_argument("--start-index", type=int, default=0, help="First point index")
    parser.add_argument("--end-index", type=int, default=24, help="Last point index")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("calib_tool_tf-robot1_to_robot2_rotation.json"),
        help="Output JSON path",
    )
    args = parser.parse_args()

    points_r1 = []
    points_r2 = []
    r1_rotations = []
    r2_rotations = []
    used_indices = []

    for idx in range(args.start_index, args.end_index + 1):
        p1_path = args.points_dir / f"calib_tool_tf-P{idx}.json"
        p2_path = args.points_dir / f"calib_tool_tf-P{idx}-R2.json"

        if not p1_path.exists() or not p2_path.exists():
            raise FileNotFoundError(f"Missing pair for P{idx}: {p1_path.name}, {p2_path.name}")

        p1_xyz, r1_rotation = load_pose(p1_path)
        p2_xyz, r2_rotation = load_pose(p2_path)

        points_r1.append(p1_xyz)
        points_r2.append(p2_xyz)
        r1_rotations.append(r1_rotation)
        r2_rotations.append(r2_rotation)
        used_indices.append(idx)

    points_r1_np = np.array(points_r1, dtype=float)
    points_r2_np = np.array(points_r2, dtype=float)

    if points_r1_np.shape[0] < 3:
        raise RuntimeError("Need at least 3 points to estimate a 3D rigid transform.")

    position_transform, position_rotation, position_translation, singular_values, point_errors, rmse = (
        compute_rigid_transform_svd(points_r1_np, points_r2_np)
    )

    # Estimate an orientation correction from the calibration 6DOF poses.
    # For each pair, compute the relative rotation R_r1_to_r2 and average them.
    relative_rotations = [r2 @ r1.T for r1, r2 in zip(r1_rotations, r2_rotations)]
    rotation_correction = average_rotation(relative_rotations)

    final_rotation = rotation_correction @ position_rotation
    final_translation = points_r2_np.mean(axis=0) - final_rotation @ points_r1_np.mean(axis=0)
    final_transform = compose_transform(final_rotation, final_translation)

    output_data = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "method": "SVD_Kabsch_plus_6dof_rotation_correction",
        "input": {
            "points_dir": str(args.points_dir),
            "indices": used_indices,
            "robot1_pattern": "calib_tool_tf-P{i}.json",
            "robot2_pattern": "calib_tool_tf-P{i}-R2.json",
            "position_only": False,
            "rotation_correction_from_calibration": True,
        },
        "transform_robot1_to_robot2": {
            "position_only_matrix_4x4": position_transform.tolist(),
            "rotation_correction_3x3": rotation_correction.tolist(),
            "matrix_4x4": final_transform.tolist(),
            "rotation_3x3": final_rotation.tolist(),
            "translation_xyz": final_translation.tolist(),
        },
        "fit_metrics": {
            "num_points": len(used_indices),
            "rmse_m": rmse,
            "max_error_m": float(np.max(point_errors)),
            "mean_error_m": float(np.mean(point_errors)),
            "singular_values": singular_values.tolist(),
        },
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as outfile:
        json.dump(output_data, outfile, indent=2)

    print(f"Saved Robot1->Robot2 transform with rotation correction to {args.output}")
    print(f"Points used: {len(used_indices)} ({used_indices[0]}..{used_indices[-1]})")
    print(f"RMSE (position-only alignment): {rmse:.6f} m")


if __name__ == "__main__":
    main()