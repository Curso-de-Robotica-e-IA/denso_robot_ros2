"""Record a ROS image topic as a compact, constant-frame-rate video."""

import json
import os
from datetime import datetime
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DebugVideoRecorder(Node):
    """Write selected frames from an image topic to an MP4 file."""

    def __init__(self):
        super().__init__('debug_video_recorder')

        self.declare_parameter('image_topic', 'debug_image')
        self.declare_parameter(
            'output_dir',
            '/root/denso_ws/src/denso_robot_ros2/debug_videos',
        )
        self.declare_parameter('filename_prefix', 'debug_image')
        self.declare_parameter('record_fps', 15.0)
        self.declare_parameter('scale', 1.0)
        self.declare_parameter('codec', 'avc1')

        self._image_topic = str(self.get_parameter('image_topic').value)
        self._output_dir = Path(
            os.path.expanduser(str(self.get_parameter('output_dir').value))
        )
        self._prefix = str(self.get_parameter('filename_prefix').value)
        self._fps = float(self.get_parameter('record_fps').value)
        self._scale = float(self.get_parameter('scale').value)
        self._requested_codec = str(self.get_parameter('codec').value)

        if self._fps <= 0.0:
            raise ValueError('record_fps must be greater than zero')
        if not 0.1 <= self._scale <= 1.0:
            raise ValueError('scale must be between 0.1 and 1.0')
        if len(self._requested_codec) != 4:
            raise ValueError('codec must contain exactly four characters')

        self._bridge = CvBridge()
        self._writer = None
        self._video_path = None
        self._metadata_path = None
        self._codec = None
        self._frame_size = None
        self._started_at = None
        self._first_stamp_ns = None
        self._last_stamp_ns = None
        self._received_frames = 0
        self._written_frames = 0
        self._skipped_frames = 0
        self._next_frame_time = None
        self._last_metadata_write = 0.0
        self._warned_size_change = False

        self.create_subscription(
            Image,
            self._image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Waiting for images on {self._image_topic}; recording at '
            f'{self._fps:g} FPS'
        )

    @staticmethod
    def _message_stamp_ns(message):
        stamp = message.header.stamp
        value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        return value or None

    def _open_writer(self, frame):
        self._output_dir.mkdir(parents=True, exist_ok=True)
        # Include microseconds so rapid restarts can never reuse a filename.
        timestamp = datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
        self._video_path = self._output_dir / f'{self._prefix}_{timestamp}.mp4'
        self._metadata_path = self._video_path.with_suffix('.json')

        height, width = frame.shape[:2]
        width = max(2, int(round(width * self._scale)))
        height = max(2, int(round(height * self._scale)))
        # Video encoders generally require even dimensions.
        width -= width % 2
        height -= height % 2
        self._frame_size = (width, height)

        codecs = [self._requested_codec]
        if self._requested_codec != 'mp4v':
            codecs.append('mp4v')
        for codec in codecs:
            writer = cv2.VideoWriter(
                str(self._video_path),
                cv2.VideoWriter_fourcc(*codec),
                self._fps,
                self._frame_size,
            )
            if writer.isOpened():
                self._writer = writer
                self._codec = codec
                break
            writer.release()

        if self._writer is None:
            raise RuntimeError(
                f'could not open a video encoder for {self._video_path}'
            )

        self._started_at = datetime.now().astimezone().isoformat()
        self.get_logger().info(
            f'Recording {self._frame_size[0]}x{self._frame_size[1]} '
            f'{self._codec} video to {self._video_path}'
        )
        self._write_metadata(final=False)

    def _write_metadata(self, final):
        if self._metadata_path is None:
            return
        metadata = {
            'video': self._video_path.name,
            'image_topic': self._image_topic,
            'codec': self._codec,
            'width': self._frame_size[0],
            'height': self._frame_size[1],
            'record_fps': self._fps,
            'scale': self._scale,
            'started_at': self._started_at,
            'finished_at': (
                datetime.now().astimezone().isoformat() if final else None
            ),
            'received_frames': self._received_frames,
            'written_frames': self._written_frames,
            'skipped_frames': self._skipped_frames,
            'first_ros_stamp_ns': self._first_stamp_ns,
            'last_ros_stamp_ns': self._last_stamp_ns,
            'complete': final,
        }
        temporary = self._metadata_path.with_suffix('.json.tmp')
        temporary.write_text(
            json.dumps(metadata, indent=2) + '\n', encoding='utf-8'
        )
        temporary.replace(self._metadata_path)
        self._last_metadata_write = time.monotonic()

    def _on_image(self, message):
        self._received_frames += 1
        now = time.monotonic()
        frame_period = 1.0 / self._fps
        timing_tolerance = 0.25 * frame_period
        if (
            self._next_frame_time is not None
            and now < self._next_frame_time - timing_tolerance
        ):
            self._skipped_frames += 1
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8'
            )
            if self._writer is None:
                self._open_writer(frame)
            if (frame.shape[1], frame.shape[0]) != self._frame_size:
                frame = cv2.resize(
                    frame, self._frame_size, interpolation=cv2.INTER_AREA
                )
                if self._scale == 1.0 and not self._warned_size_change:
                    self.get_logger().warning(
                        'Input image size changed; resizing to the first frame'
                    )
                    self._warned_size_change = True
            self._writer.write(frame)
        except (CvBridgeError, OSError, RuntimeError, ValueError) as error:
            self.get_logger().error(f'Could not record debug frame: {error}')
            return

        stamp_ns = self._message_stamp_ns(message)
        if self._first_stamp_ns is None:
            self._first_stamp_ns = stamp_ns
        self._last_stamp_ns = stamp_ns
        self._written_frames += 1
        if self._next_frame_time is None:
            self._next_frame_time = now + frame_period
        else:
            self._next_frame_time += frame_period
            if self._next_frame_time <= now:
                self._next_frame_time = now + frame_period
        if now - self._last_metadata_write >= 5.0:
            self._write_metadata(final=False)

    def close(self):
        """Flush the video and mark its metadata as complete."""
        if self._writer is None:
            return
        self._writer.release()
        self._writer = None
        self._write_metadata(final=True)
        size_mb = self._video_path.stat().st_size / (1024.0 * 1024.0)
        self.get_logger().info(
            f'Saved {self._written_frames} frames to {self._video_path} '
            f'({size_mb:.1f} MiB)'
        )

    def destroy_node(self):
        self.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DebugVideoRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
