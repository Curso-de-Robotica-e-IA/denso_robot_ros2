#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np


def load_transform_matrix(transform_json: Path) -> np.ndarray:
    with transform_json.open("r", encoding="utf-8") as infile:
        data = json.load(infile)

    matrix = np.array(data["transform_robot1_to_robot2"]["matrix_4x4"], dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f"Expected 4x4 transform matrix, got shape {matrix.shape}")

    return matrix


def load_position_from_point_file(path: Path) -> np.ndarray:
    with path.open("r", encoding="utf-8") as infile:
        data = json.load(infile)

    position = data["position"]
    return np.array([position["x"], position["y"], position["z"]], dtype=float)


def load_pose_from_point_file(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with path.open("r", encoding="utf-8") as infile:
        data = json.load(infile)

    if "position" not in data or "orientation" not in data:
        raise KeyError(f"Missing position/orientation in {path}")

    position = data["position"]
    orientation = data["orientation"]
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


def rotation_matrix_to_quaternion(rotation: np.ndarray) -> dict[str, float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s

    quaternion = np.array([x, y, z, w], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    return {
        "x": float(quaternion[0]),
        "y": float(quaternion[1]),
        "z": float(quaternion[2]),
        "w": float(quaternion[3]),
    }


def transform_point(matrix_4x4: np.ndarray, point_xyz: np.ndarray) -> np.ndarray:
    rotation = matrix_4x4[:3, :3]
    translation = matrix_4x4[:3, 3]
    return rotation @ point_xyz + translation


def transform_rotation(matrix_4x4: np.ndarray, rotation_r1: np.ndarray) -> np.ndarray:
    transform_rotation_matrix = matrix_4x4[:3, :3]
    return transform_rotation_matrix @ rotation_r1


def build_input_points_from_dir(
    points_dir: Path,
    start_index: int,
    end_index: int,
    robot1_pattern: str,
    include_rotation: bool,
) -> list[dict[str, Any]]:
    points: list[dict[str, Any]] = []

    for idx in range(start_index, end_index + 1):
        point_path = points_dir / robot1_pattern.format(i=idx)
        if not point_path.exists():
            raise FileNotFoundError(f"Missing Robot1 point file for P{idx}: {point_path}")

        if include_rotation:
            p_xyz, p_rotation = load_pose_from_point_file(point_path)
        else:
            p_xyz = load_position_from_point_file(point_path)
            p_rotation = None
        points.append(
            {
                "label": f"P{idx}",
                "source_file": str(point_path),
                "position_r1": p_xyz,
                "rotation_r1": p_rotation,
            }
        )

    return points


def build_input_points_from_json(input_json: Path, include_rotation: bool) -> list[dict[str, Any]]:
    with input_json.open("r", encoding="utf-8") as infile:
        data = json.load(infile)

    def parse_pos(position_obj: dict[str, Any]) -> np.ndarray:
        return np.array(
            [position_obj["x"], position_obj["y"], position_obj["z"]],
            dtype=float,
        )

    def parse_rotation(item: dict[str, Any]) -> np.ndarray | None:
        if not include_rotation:
            return None
        if "orientation" not in item:
            raise ValueError("Rotation transform requested, but input is missing 'orientation'")
        quaternion = item["orientation"].get("quaternion")
        if quaternion is None:
            raise ValueError("Rotation transform requested, but input is missing 'orientation.quaternion'")
        return quaternion_to_rotation_matrix(quaternion["x"], quaternion["y"], quaternion["z"], quaternion["w"])

    points: list[dict[str, Any]] = []

    if isinstance(data, dict) and "position" in data:
        points.append(
            {
                "label": "point_0",
                "source_file": str(input_json),
                "position_r1": parse_pos(data["position"]),
                "rotation_r1": parse_rotation(data),
            }
        )
        return points

    if isinstance(data, dict) and "points" in data and isinstance(data["points"], list):
        for idx, item in enumerate(data["points"]):
            if "position" not in item:
                raise ValueError(f"points[{idx}] is missing 'position'")
            points.append(
                {
                    "label": item.get("label", f"point_{idx}"),
                    "source_file": str(input_json),
                    "position_r1": parse_pos(item["position"]),
                    "rotation_r1": parse_rotation(item),
                }
            )
        return points

    if isinstance(data, list):
        for idx, item in enumerate(data):
            if not isinstance(item, dict) or "position" not in item:
                raise ValueError("List input must contain objects with a 'position' field")
            points.append(
                {
                    "label": item.get("label", f"point_{idx}"),
                    "source_file": str(input_json),
                    "position_r1": parse_pos(item["position"]),
                    "rotation_r1": parse_rotation(item),
                }
            )
        return points

    raise ValueError(
        "Unsupported input JSON format. Use either: "
        "{'position': {...}}, {'points': [{'position': {...}}, ...]}, or a list of point objects."
    )


def build_input_points_from_cli(x: float, y: float, z: float) -> list[dict[str, Any]]:
    return [
        {
            "label": "cli_point",
            "source_file": "cli",
            "position_r1": np.array([x, y, z], dtype=float),
            "rotation_r1": None,
        }
    ]


def point_to_dict(point_xyz: np.ndarray) -> dict[str, float]:
    return {
        "x": float(point_xyz[0]),
        "y": float(point_xyz[1]),
        "z": float(point_xyz[2]),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Transform Robot1 points into Robot2 coordinates using a precomputed "
            "Robot1->Robot2 4x4 transform matrix."
        )
    )
    parser.add_argument(
        "--transform-json",
        type=Path,
        default=Path("calib_tool_tf-robot1_to_robot2.json"),
        help="Path to JSON generated by compute_robot1_to_robot2_tf.py",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("robot1_points_in_robot2_frame.json"),
        help="Output JSON path",
    )

    source_group = parser.add_mutually_exclusive_group(required=True)
    source_group.add_argument(
        "--input-json",
        type=Path,
        help="JSON with Robot1 points",
    )
    source_group.add_argument(
        "--points-dir",
        type=Path,
        help="Directory containing calib_tool_tf-P{i}.json files for Robot1",
    )
    source_group.add_argument(
        "--point-xyz",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Single Robot1 point in meters",
    )

    parser.add_argument("--start-index", type=int, default=0, help="First point index when using --points-dir")
    parser.add_argument("--end-index", type=int, default=24, help="Last point index when using --points-dir")
    parser.add_argument(
        "--robot1-pattern",
        type=str,
        default="calib_tool_tf-P{i}.json",
        help="Filename pattern for Robot1 points when using --points-dir",
    )
    parser.add_argument(
        "--include-rotation",
        action="store_true",
        help="Also transform the input orientation matrix using the Robot1->Robot2 rotation",
    )

    args = parser.parse_args()

    matrix = load_transform_matrix(args.transform_json)

    if args.input_json is not None:
        input_points = build_input_points_from_json(args.input_json, args.include_rotation)
    elif args.points_dir is not None:
        input_points = build_input_points_from_dir(
            points_dir=args.points_dir,
            start_index=args.start_index,
            end_index=args.end_index,
            robot1_pattern=args.robot1_pattern,
            include_rotation=args.include_rotation,
        )
    else:
        cli_xyz = args.point_xyz
        if cli_xyz is None:
            raise RuntimeError("No point source provided")
        input_points = build_input_points_from_cli(cli_xyz[0], cli_xyz[1], cli_xyz[2])

    output_points = []
    moveit_position_targets = []
    moveit_orientation_targets = []

    for item in input_points:
        p_r1 = item["position_r1"]
        p_r2 = transform_point(matrix, p_r1)

        output_item = {
            "label": item["label"],
            "position_robot2": point_to_dict(p_r2),
        }

        if args.include_rotation:
            if item["rotation_r1"] is None:
                raise ValueError(
                    "Rotation transform requested, but the selected input source does not include orientation data"
                )

            r_r2 = transform_rotation(matrix, item["rotation_r1"])
            quaternion_r2 = rotation_matrix_to_quaternion(r_r2)
            output_item["orientation_robot2_quaternion"] = quaternion_r2
            moveit_orientation_targets.append(
                [
                    quaternion_r2["x"],
                    quaternion_r2["y"],
                    quaternion_r2["z"],
                    quaternion_r2["w"],
                ]
            )

        output_points.append(output_item)
        moveit_position_targets.append([float(p_r2[0]), float(p_r2[1]), float(p_r2[2])])

    output_data = {
        "transform_json": str(args.transform_json),
        "num_points": len(output_points),
        "include_rotation": args.include_rotation,
        "points": output_points,
        "moveit_position_targets_xyz": moveit_position_targets,
    }

    if args.include_rotation:
        output_data["moveit_orientation_targets_xyzw"] = moveit_orientation_targets

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as outfile:
        json.dump(output_data, outfile, indent=2)

    print(f"Saved transformed points to {args.output}")
    print(f"Transformed points: {len(output_points)}")
    print("MoveIt position targets (x,y,z) [m]:")
    for target in moveit_position_targets:
        print(f"  {target}")
    if args.include_rotation:
        print("MoveIt orientation targets (x,y,z,w):")
        for target in moveit_orientation_targets:
            print(f"  {target}")


if __name__ == "__main__":
    main()
