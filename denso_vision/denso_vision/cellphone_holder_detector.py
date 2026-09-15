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

        self.declare_parameter('sim', False)
        self.declare_parameter('image_topic', '/basic_camera')
        self.declare_parameter(
            'realsense_frame_id', 'camera_color_optical_frame'
        )
        self.declare_parameter('realsense_serial', '')
        self.declare_parameter('realsense_width', 1280)
        self.declare_parameter('realsense_height', 720)
        self.declare_parameter('realsense_fps', 30)
        self.declare_parameter(
            'holder_frame_id', 'cellphone_holder_tags_frame'
        )
        self.declare_parameter('tag_ids', [1, 2, 3, 4])
        self.declare_parameter('tag_size_mm', 21.5)
        self.declare_parameter('top_spacing_mm', 95.2)
        self.declare_parameter('bottom_spacing_mm', 99.8)
        self.declare_parameter('side_spacing_mm', 207.5)
        self.declare_parameter('target_pixel_x', -1.0)
        self.declare_parameter('target_pixel_y', -1.0)
        self.declare_parameter('homography_ransac_threshold_mm', 2.0)
        self.declare_parameter('publish_debug_image', True)

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
        canonical_corners = self._geometry.tag_corners()
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

        image_topic = self.get_parameter('image_topic').value
        self._sim = self.get_parameter('sim').value
        if self._sim:
            self.create_subscription(
                Image,
                image_topic,
                self._image_callback,
                qos_profile_sensor_data,
            )
        else:
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

        source = image_topic if self._sim else 'RealSense D405'
        self.get_logger().info(
            f'Looking for AprilTag 36h10 IDs {self._tag_ids} on {source}'
        )

    def destroy_node(self):
        """Stop the RealSense pipeline before destroying the ROS node."""
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None
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

    def _process_image(
        self, message: Image, color_image: np.ndarray
    ) -> None:
        """Detect tags and publish homography results for one BGR image."""
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray_image)
        ordered_points = self._ordered_detections(corners, ids)

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
                inliers = inlier_mask.ravel().astype(bool)
                errors = np.linalg.norm(
                    projected[inliers] - self._plane_points[inliers], axis=1
                )
                rmse_mm = 1000.0 * np.sqrt(np.mean(errors ** 2))
                self._rmse_publisher.publish(Float64(data=float(rmse_mm)))

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
                cv2.drawMarker(
                    color_image,
                    tuple(np.rint(target_pixel).astype(int)),
                    (0, 0, 255),
                    cv2.MARKER_CROSS,
                    24,
                    2,
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
