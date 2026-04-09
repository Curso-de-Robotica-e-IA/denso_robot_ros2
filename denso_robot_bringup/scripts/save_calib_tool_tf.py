#!/usr/bin/env python3

import argparse
import gc
import json
import math
from datetime import datetime, timezone

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_euler_xyz(x: float, y: float, z: float, w: float):
    """Convert quaternion (x, y, z, w) to roll/pitch/yaw in radians."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class CalibToolTfSaver(Node):
    def __init__(self, source_frame: str, target_frame: str):
        super().__init__("calib_tool_tf_saver")
        self.source_frame = source_frame
        self.target_frame = target_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

    def close(self) -> None:
        """Stop TransformListener thread before shutting down rclpy context."""
        listener = getattr(self, "tf_listener", None)
        if listener is None:
            return

        self.tf_listener = None

        if hasattr(listener, "executor") and hasattr(listener, "dedicated_listener_thread"):
            listener.executor.shutdown()
            listener.dedicated_listener_thread.join(timeout=2.0)

        listener.unregister()
        del listener
        gc.collect()

    def lookup_transform(self):
        transform = self.tf_buffer.lookup_transform(
            self.source_frame,
            self.target_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=10.0),
        )

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        roll, pitch, yaw = quaternion_to_euler_xyz(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )

        return {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "source_frame": self.source_frame,
            "target_frame": self.target_frame,
            "position": {
                "x": translation.x,
                "y": translation.y,
                "z": translation.z,
            },
            "orientation": {
                "quaternion": {
                    "x": rotation.x,
                    "y": rotation.y,
                    "z": rotation.z,
                    "w": rotation.w,
                },
                "euler_rpy_rad": {
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw,
                },
                "euler_rpy_deg": {
                    "roll": math.degrees(roll),
                    "pitch": math.degrees(pitch),
                    "yaw": math.degrees(yaw),
                },
            },
        }


def wait_for_transform(node: CalibToolTfSaver, timeout_sec: float):
    deadline = node.get_clock().now() + Duration(seconds=timeout_sec)
    while rclpy.ok() and node.get_clock().now() < deadline:
        if node.tf_buffer.can_transform(
            node.source_frame,
            node.target_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.2),
        ):
            return True
        rclpy.spin_once(node, timeout_sec=0.1)
    return False


def main():
    parser = argparse.ArgumentParser(
        description="Read calib tool transform from TF2 and save it to JSON."
    )
    parser.add_argument(
        "--source-frame",
        default="world",
        help="Parent/source frame (default: world)",
    )
    parser.add_argument(
        "--target-frame",
        default="calib_link",
        help="Child/target frame (default: calib_link)",
    )
    parser.add_argument(
        "--output",
        default="calib_tool_tf.json",
        help="Output JSON path (default: calib_tool_tf.json)",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for TF availability (default: 10.0)",
    )

    args = parser.parse_args()

    rclpy.init()
    node = CalibToolTfSaver(args.source_frame, args.target_frame)

    try:
        if not wait_for_transform(node, args.wait_timeout):
            node.get_logger().error(
                f"Timed out waiting for TF {args.source_frame} -> {args.target_frame}"
            )
            raise RuntimeError("Transform not available")

        current = node.lookup_transform()
        print("Current transform:")
        print(json.dumps(current, indent=2))

        input("Press Enter to save the current transform to JSON... ")

        # Re-read after keypress so the saved data reflects the latest frame value.
        to_save = node.lookup_transform()
        with open(args.output, "w", encoding="utf-8") as outfile:
            json.dump(to_save, outfile, indent=2)

        print(f"Saved transform to: {args.output}")

    except (TransformException, RuntimeError) as error:
        node.get_logger().error(str(error))
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
