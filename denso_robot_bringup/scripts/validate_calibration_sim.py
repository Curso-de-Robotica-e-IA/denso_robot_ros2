#!/usr/bin/python3
"""
Validate the calibration pipeline using simulation.

This script:
1. Listens to the TF tree to get the pose of left_J6 in left_base_link coordinates
2. Gets the transformation from left_base_link to right_base_link (calibration matrix)
3. Transforms the left_J6 position to right_base_link frame
4. Saves in JSON format compatible with send_moveit_points_from_json

The output JSON contains:
- position_robot1 (left_base_link): The position of left_J6 relative to left robot base
- position_robot2_transformed (right_base_link): The position that right_J6 should reach
  to be at the same physical location as left_J6 (in the dual robot independent frame setup)

Usage:
    python3 validate_calibration_sim.py --output output.json
"""

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformException, TransformListener


def transform_4x4_from_tf(transform) -> np.ndarray:
    """Convert a TF StampedTransform to a 4x4 homogeneous transformation matrix."""
    translation = transform.transform.translation
    rotation = transform.transform.rotation

    # Use scipy Rotation for quaternion to rotation matrix conversion
    quat = [rotation.x, rotation.y, rotation.z, rotation.w]
    rotation_matrix = Rotation.from_quat(quat).as_matrix()

    matrix_4x4 = np.eye(4, dtype=float)
    matrix_4x4[:3, :3] = rotation_matrix
    matrix_4x4[:3, 3] = [translation.x, translation.y, translation.z]

    return matrix_4x4


def transform_point(matrix_4x4: np.ndarray, point_xyz: np.ndarray) -> np.ndarray:
    """Transform a 3D point using a 4x4 transformation matrix."""
    point_homogeneous = np.append(point_xyz, 1.0)
    transformed = matrix_4x4 @ point_homogeneous
    return transformed[:3]


def rotation_matrix_to_quaternion_dict(rotation_matrix: np.ndarray) -> dict[str, float]:
    """Convert 3x3 rotation matrix to quaternion dict (x, y, z, w)."""
    rotation = Rotation.from_matrix(rotation_matrix)
    quat = rotation.as_quat()  # Returns [x, y, z, w]
    return {
        "x": float(quat[0]),
        "y": float(quat[1]),
        "z": float(quat[2]),
        "w": float(quat[3]),
    }


class CalibrationSimValidator(Node):
    def __init__(self, output_file: str):
        super().__init__("calibration_sim_validator")
        self.output_file = Path(output_file)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.get_logger().info(f"Listening to TF tree, will save to {self.output_file}")

    def validate(self) -> bool:
        """
        Perform the validation (simulating real dual-robot scenario):
        1. Get left_J6 position relative to left_base_link
        2. Get transformation from left_base_link to right_base_link (the calibration matrix)
        3. Transform left_J6 position to right_base_link frame
        4. Save to JSON with both positions (left_base_link and right_base_link)
        
        This mimics the real scenario where each robot has its own coordinate system,
        and the calibration matrix transforms coordinates from one robot to the other.
        """
        try:
            # Step 1: Get left_J6 position relative to left_base_link
            self.get_logger().info("Step 1: Getting pose of left_J6 relative to left_base_link...")
            transform_left_j6_to_base = self.tf_buffer.lookup_transform(
                "left_base_link",
                "left_J6",
                rclpy.time.Time(),
                timeout=Duration(seconds=10.0),
            )

            position_left_j6_base = transform_left_j6_to_base.transform.translation
            point_left_base = np.array(
                [position_left_j6_base.x, position_left_j6_base.y, position_left_j6_base.z],
                dtype=float,
            )
            self.get_logger().info(f"✓ left_J6 position in left_base_link frame: {point_left_base}")

            # Get orientation of left_J6 in left_base_link frame
            rotation_left_j6_base = transform_left_j6_to_base.transform.rotation
            rotation_matrix_left_j6_base = Rotation.from_quat(
                [rotation_left_j6_base.x, rotation_left_j6_base.y, rotation_left_j6_base.z, rotation_left_j6_base.w]
            ).as_matrix()

            # Step 2: Get transformation from left_base_link to right_base_link (calibration matrix)
            self.get_logger().info("Step 2: Getting calibration transformation: left_base_link -> right_base_link...")
            transform_left_base_to_right_base = self.tf_buffer.lookup_transform(
                "right_base_link",
                "left_base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=10.0),
            )

            matrix_left_base_to_right_base = transform_4x4_from_tf(transform_left_base_to_right_base)
            self.get_logger().info(
                f"✓ Calibration transform matrix (left_base_link -> right_base_link):\n{matrix_left_base_to_right_base}"
            )

            # Step 3: Transform left_J6 from left_base_link frame to right_base_link frame
            # This tells us: "if left_J6 is at this position in left frame, 
            # right_J6 needs to be at this position in right frame to reach the same physical location"
            point_right_base = transform_point(matrix_left_base_to_right_base, point_left_base)
            self.get_logger().info(f"✓ left_J6 position transformed to right_base_link frame: {point_right_base}")

            # Transform orientation
            transform_rotation_matrix = matrix_left_base_to_right_base[:3, :3]
            rotation_matrix_right_j6 = transform_rotation_matrix @ rotation_matrix_left_j6_base
            quat_right_j6 = rotation_matrix_to_quaternion_dict(rotation_matrix_right_j6)

            # Step 4: Save to JSON
            output_data = {
                "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                "description": "Validation of calibration pipeline through simulation (dual robot frames)",
                "source_frames": "left_J6 (in left_base_link frame) -> right_base_link (right robot)",
                "transformer": "left_base_link -> right_base_link (calibration matrix)",
                "points": [
                    {
                        "label": "left_J6_current_pose",
                        "position_robot1": {
                            "x": float(point_left_base[0]),
                            "y": float(point_left_base[1]),
                            "z": float(point_left_base[2]),
                        },
                        "position_robot1_frame": "left_base_link",
                        "position_robot2_transformed": {
                            "x": float(point_right_base[0]),
                            "y": float(point_right_base[1]),
                            "z": float(point_right_base[2]),
                        },
                        "position_robot2_frame": "right_base_link",
                        "orientation_robot2_transformed_quaternion": quat_right_j6,
                    }
                ],
                "transform_matrix_4x4_left_base_to_right_base": matrix_left_base_to_right_base.tolist(),
            }

            with self.output_file.open("w", encoding="utf-8") as f:
                json.dump(output_data, f, indent=2)

            self.get_logger().info(f"✓ Validation successful! Output saved to {self.output_file}")
            return True

        except TransformException as e:
            self.get_logger().error(f"✗ Transform lookup failed: {e}")
            return False
        except Exception as e:
            self.get_logger().error(f"✗ Error during validation: {e}")
            return False


def main():
    parser = argparse.ArgumentParser(
        description="Validate calibration pipeline through simulation"
    )
    parser.add_argument(
        "--output",
        type=str,
        default="calibration_validation.json",
        help="Output JSON file path",
    )
    args = parser.parse_args()

    rclpy.init()
    node = CalibrationSimValidator(args.output)
    success = node.validate()
    node.destroy_node()
    return 0 if success else 1


if __name__ == "__main__":
    exit(main())
