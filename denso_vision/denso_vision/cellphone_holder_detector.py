"""ROS 2 node for detecting the cellphone holder's four AprilTags."""

from typing import Dict, Optional

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PolygonStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension

from denso_vision.holder_geometry import HolderGeometry, transform_pixel


class CellphoneHolderDetector(Node):
    """Estimate pixel-to-holder-plane coordinates from tag IDs 1-4."""

    def __init__(self) -> None:
        """Create subscriptions, publishers, detector, and holder geometry."""
        super().__init__('cellphone_holder_detector')

        self.declare_parameter('image_source', 'realsense')
        self.declare_parameter('image_topic', '/basic_camera')
        self.declare_parameter('video_path', '')
        self.declare_parameter('video_loop', True)
        self.declare_parameter('video_fps', 0.0)
        self.declare_parameter(
            'realsense_frame_id', 'camera_color_optical_frame'
        )
        self.declare_parameter('realsense_serial', '')
        self.declare_parameter('realsense_width', 1280)
        self.declare_parameter('realsense_height', 720)
        self.declare_parameter('realsense_fps', 30)
        self.declare_parameter(
            'holder_frame_id', 'right_cellphone_holder_tags_frame'
        )
        self.declare_parameter('tag_ids', [1, 2, 3, 4])
        self.declare_parameter('tag_size_mm', 21.5)
        self.declare_parameter('top_spacing_mm', 95.2)
        self.declare_parameter('bottom_spacing_mm', 99.8)
        self.declare_parameter('side_spacing_mm', 207.5)
        self.declare_parameter('target_pixel_x', -1.0)
        self.declare_parameter('target_pixel_y', -1.0)
        self.declare_parameter('homography_ransac_threshold_mm', 4.0)
        self.declare_parameter('fallback_binary_threshold', 90)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('detect_colored_dots', True)
        self.declare_parameter('min_dot_radius_px', 12.0)
        self.declare_parameter('dot_roi_half_width_mm', 50.0)
        self.declare_parameter('dot_roi_half_height_mm', 100.0)

        self._tag_ids = tuple(
            int(value) for value in self.get_parameter('tag_ids').value
        )
        if len(self._tag_ids) != 4 or len(set(self._tag_ids)) != 4:
            raise ValueError('tag_ids must contain four distinct IDs')

        to_metres = 0.001
        self._geometry = HolderGeometry(
            tag_size=self.get_parameter('tag_size_mm').value * to_metres,
            top_spacing=(
                self.get_parameter('top_spacing_mm').value * to_metres
            ),
            bottom_spacing=(
                self.get_parameter('bottom_spacing_mm').value * to_metres
            ),
            side_spacing=(
                self.get_parameter('side_spacing_mm').value * to_metres
            ),
        )
        # Evaluate once at startup so invalid measurements fail clearly.
        _ = self._geometry.height
        canonical_corners = self._geometry.detector_corners()
        self._plane_points = np.concatenate([
            canonical_corners[index]
            for index in (1, 2, 3, 4)
        ]).astype(np.float64)

        dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h10
        )
        if hasattr(cv2.aruco, 'DetectorParameters'):
            detector_parameters = cv2.aruco.DetectorParameters()
        else:
            detector_parameters = cv2.aruco.DetectorParameters_create()
        detector_parameters.cornerRefinementMethod = \
            cv2.aruco.CORNER_REFINE_SUBPIX
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self._detector = cv2.aruco.ArucoDetector(
                dictionary, detector_parameters
            )
            self._detect = self._detector.detectMarkers
        else:
            self._detect = lambda image: cv2.aruco.detectMarkers(
                image, dictionary, parameters=detector_parameters
            )

        self._bridge = CvBridge()
        self._requested_pixel: Optional[np.ndarray] = None
        self._holder_frame_id = self.get_parameter('holder_frame_id').value
        self._ransac_threshold = (
            self.get_parameter('homography_ransac_threshold_mm').value
            * to_metres
        )
        self._publish_debug = self.get_parameter('publish_debug_image').value
        self._pipeline = None
        self._video_capture = None
        self._video_timer = None

        image_topic = self.get_parameter('image_topic').value
        self._image_source = self.get_parameter('image_source').value
        if self._image_source == 'topic':
            self.create_subscription(
                Image,
                image_topic,
                self._image_callback,
                qos_profile_sensor_data,
            )
        elif self._image_source == 'realsense':
            try:
                import pyrealsense2 as rs
            except ImportError as error:
                raise RuntimeError(
                    'sim=false requires the pyrealsense2 Python package'
                ) from error
            self._rs = rs
            self._pipeline = rs.pipeline()
            realsense_config = rs.config()
            realsense_serial = self.get_parameter(
                'realsense_serial'
            ).value
            if realsense_serial:
                realsense_config.enable_device(realsense_serial)
            realsense_config.enable_stream(
                rs.stream.color,
                self.get_parameter('realsense_width').value,
                self.get_parameter('realsense_height').value,
                rs.format.bgr8,
                self.get_parameter('realsense_fps').value,
            )
            self._pipeline.start(realsense_config)
            self.create_timer(1.0 / 30.0, self._realsense_callback)
        elif self._image_source == 'video':
            video_path = self.get_parameter('video_path').value
            if not video_path:
                raise ValueError('image_source=video requires video_path')
            self._video_capture = cv2.VideoCapture(video_path)
            if not self._video_capture.isOpened():
                raise RuntimeError(f'Could not open video: {video_path}')
            recorded_fps = self._video_capture.get(cv2.CAP_PROP_FPS)
            requested_fps = self.get_parameter('video_fps').value
            playback_fps = requested_fps if requested_fps > 0.0 else recorded_fps
            if playback_fps <= 0.0:
                playback_fps = 30.0
            self._video_timer = self.create_timer(
                1.0 / playback_fps, self._video_callback
            )
        else:
            raise ValueError(
                'image_source must be one of: video, topic, realsense'
            )
        self.create_subscription(
            PointStamped, 'target_pixel', self._target_pixel_callback, 10
        )

        self._corners_publisher = self.create_publisher(
            PolygonStamped, 'tag_corners', 10
        )
        self._homography_publisher = self.create_publisher(
            Float64MultiArray, 'homography', 10
        )
        self._rmse_publisher = self.create_publisher(
            Float64, 'homography_rmse_mm', 10
        )
        self._point_publisher = self.create_publisher(
            PointStamped, 'target_point', 10
        )
        self._debug_publisher = self.create_publisher(Image, 'debug_image', 10)
        self._dot_publishers = {
            color: self.create_publisher(PointStamped, f'dots/{color}', 10)
            for color in ('red', 'blue', 'green')
        }

        source = {
            'topic': image_topic,
            'realsense': 'RealSense D405',
            'video': self.get_parameter('video_path').value,
        }[self._image_source]
        self.get_logger().info(
            f'Looking for AprilTag 36h10 IDs {self._tag_ids} on {source}'
        )

    def destroy_node(self):
        """Stop the RealSense pipeline before destroying the ROS node."""
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None
        if self._video_capture is not None:
            self._video_capture.release()
            self._video_capture = None
        return super().destroy_node()

    def _realsense_callback(self) -> None:
        frames = self._pipeline.poll_for_frames()
        if not frames:
            return
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        if color_frame.profile.format() == self._rs.format.rgb8:
            color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.get_parameter(
            'realsense_frame_id'
        ).value
        self._process_image(message, color_image)

    def _video_callback(self) -> None:
        """Read and process one BGR frame from an AVI or other OpenCV video."""
        ok, color_image = self._video_capture.read()
        if not ok and self.get_parameter('video_loop').value:
            self._video_capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, color_image = self._video_capture.read()
        if not ok:
            self.get_logger().info('Video finished')
            self._video_timer.cancel()
            return

        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.get_parameter(
            'realsense_frame_id'
        ).value
        self._process_image(message, color_image)

    def _target_pixel_callback(self, message: PointStamped) -> None:
        self._requested_pixel = np.array(
            [message.point.x, message.point.y], dtype=np.float64
        )

    def _ordered_detections(
        self, corners: list, ids: Optional[np.ndarray]
    ) -> Optional[np.ndarray]:
        if ids is None:
            return None
        by_id: Dict[int, np.ndarray] = {}
        for tag_corners, tag_id in zip(corners, ids.flatten()):
            numeric_id = int(tag_id)
            if numeric_id in self._tag_ids and numeric_id not in by_id:
                by_id[numeric_id] = np.asarray(
                    tag_corners, dtype=np.float64
                ).reshape(4, 2)
        if any(tag_id not in by_id for tag_id in self._tag_ids):
            return None
        return np.concatenate([by_id[tag_id] for tag_id in self._tag_ids])

    def _publish_corners(
        self, image_message: Image, points: np.ndarray
    ) -> None:
        message = PolygonStamped()
        message.header = image_message.header
        for x_coordinate, y_coordinate in points:
            point = Point32()
            point.x = float(x_coordinate)
            point.y = float(y_coordinate)
            point.z = 0.0
            message.polygon.points.append(point)
        self._corners_publisher.publish(message)

    def _publish_homography(self, homography: np.ndarray) -> None:
        message = Float64MultiArray()
        message.layout.dim = [
            MultiArrayDimension(label='rows', size=3, stride=9),
            MultiArrayDimension(label='columns', size=3, stride=3),
        ]
        message.data = homography.flatten().tolist()
        self._homography_publisher.publish(message)

    def _publish_target_point(
        self, image_message: Image, homography: np.ndarray, pixel: np.ndarray
    ) -> None:
        plane_point = transform_pixel(homography, pixel)
        message = PointStamped()
        message.header.stamp = image_message.header.stamp
        message.header.frame_id = self._holder_frame_id
        message.point.x = float(plane_point[0])
        message.point.y = float(plane_point[1])
        message.point.z = 0.0
        self._point_publisher.publish(message)

    def _image_callback(self, message: Image) -> None:
        try:
            color_image = self._bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8'
            )
        except CvBridgeError as error:
            self.get_logger().error(f'Could not convert image: {error}')
            return

        self._process_image(message, color_image)

    def _detect_colored_dots(
        self, image: np.ndarray, homography: np.ndarray
    ) -> Dict[str, np.ndarray]:
        """Return coloured dots located in the holder's central physical ROI."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        ranges = {
            'red': ((0, 100, 50), (10, 255, 255), (170, 100, 50), (180, 255, 255)),
            'blue': ((100, 100, 50), (130, 255, 255)),
            'green': ((45, 80, 50), (85, 255, 255)),
        }
        dots = {}
        half_width = (
            self.get_parameter('dot_roi_half_width_mm').value * 0.001
        )
        half_height = (
            self.get_parameter('dot_roi_half_height_mm').value * 0.001
        )
        for color, bounds in ranges.items():
            mask = cv2.inRange(hsv, np.array(bounds[0]), np.array(bounds[1]))
            if len(bounds) == 4:
                mask |= cv2.inRange(hsv, np.array(bounds[2]), np.array(bounds[3]))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            candidates = []
            for contour in contours:
                (_, _), radius = cv2.minEnclosingCircle(contour)
                if radius < self.get_parameter('min_dot_radius_px').value:
                    continue
                moment = cv2.moments(contour)
                if not moment['m00']:
                    continue
                pixel = np.array([
                    moment['m10'] / moment['m00'],
                    moment['m01'] / moment['m00'],
                ])
                plane_point = transform_pixel(homography, pixel)
                if (
                    abs(plane_point[0]) <= half_width
                    and abs(plane_point[1]) <= half_height
                ):
                    candidates.append((cv2.contourArea(contour), pixel))
            if candidates:
                dots[color] = max(candidates, key=lambda item: item[0])[1]
        return dots

    def _process_image(
        self, message: Image, color_image: np.ndarray
    ) -> None:
        """Detect tags and publish homography results for one BGR image."""
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray_image)
        ordered_points = self._ordered_detections(corners, ids)

        # Gazebo's lit tag textures can lose the white border contrast needed
        # by the adaptive detector. Keep the normal path first; use a binary
        # image only when it did not recover all four known tags.
        if ordered_points is None:
            _, binary_image = cv2.threshold(
                gray_image,
                self.get_parameter('fallback_binary_threshold').value,
                255,
                cv2.THRESH_BINARY,
            )
            fallback_corners, fallback_ids, _ = self._detect(binary_image)
            fallback_points = self._ordered_detections(
                fallback_corners, fallback_ids
            )
            if fallback_points is not None:
                corners, ids, ordered_points = (
                    fallback_corners,
                    fallback_ids,
                    fallback_points,
                )

        detected_tag_ids = set() if ids is None else set(ids.flatten())
        overlay_lines = [
            f'Tags: {len(detected_tag_ids & set(self._tag_ids))}/4'
        ]

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(color_image, corners, ids)

        if ordered_points is not None:
            homography, inlier_mask = cv2.findHomography(
                ordered_points,
                self._plane_points,
                cv2.RANSAC,
                self._ransac_threshold,
            )
            if homography is not None and inlier_mask is not None:
                self._publish_corners(message, ordered_points)
                self._publish_homography(homography)

                projected = cv2.perspectiveTransform(
                    ordered_points.reshape(1, -1, 2), homography
                ).reshape(-1, 2)
                errors = np.linalg.norm(
                    projected - self._plane_points, axis=1
                )
                rmse_mm = 1000.0 * np.sqrt(np.mean(errors ** 2))
                self._rmse_publisher.publish(Float64(data=float(rmse_mm)))
                overlay_lines.append(f'H RMSE: {rmse_mm:.2f} mm')

                target_pixel = self._requested_pixel
                if target_pixel is None:
                    configured_x = self.get_parameter('target_pixel_x').value
                    configured_y = self.get_parameter('target_pixel_y').value
                    target_pixel = np.array([
                        configured_x if configured_x >= 0.0
                        else color_image.shape[1] / 2.0,
                        configured_y if configured_y >= 0.0
                        else color_image.shape[0] / 2.0,
                    ])
                self._publish_target_point(
                    message, homography, target_pixel
                )
                if self.get_parameter('detect_colored_dots').value:
                    for color, pixel in self._detect_colored_dots(
                        color_image, homography
                    ).items():
                        point = PointStamped()
                        point.header.stamp = message.header.stamp
                        point.header.frame_id = self._holder_frame_id
                        plane_point = transform_pixel(homography, pixel)
                        point.point.x, point.point.y = map(float, plane_point)
                        self._dot_publishers[color].publish(point)
                        overlay_lines.append(
                            f'{color}: {plane_point[0] * 1000:.1f}, '
                            f'{plane_point[1] * 1000:.1f} mm'
                        )
                        cv2.putText(color_image, color, tuple(np.rint(pixel).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                cv2.drawMarker(
                    color_image,
                    tuple(np.rint(target_pixel).astype(int)),
                    (0, 0, 255),
                    cv2.MARKER_CROSS,
                    24,
                    2,
                )

        for line_index, line in enumerate(overlay_lines):
            position = (12, 28 + 24 * line_index)
            cv2.putText(
                color_image, line, position, cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 0, 0), 3,
            )
            cv2.putText(
                color_image, line, position, cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255, 255, 255), 1,
            )

        if self._publish_debug:
            debug_message = self._bridge.cv2_to_imgmsg(
                color_image, encoding='bgr8'
            )
            debug_message.header = message.header
            self._debug_publisher.publish(debug_message)


def main(args=None) -> None:
    """Run the cellphone-holder detector node."""
    rclpy.init(args=args)
    node = CellphoneHolderDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
