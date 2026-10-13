"""ROS 2 node for detecting the cellphone holder and RGB/CAPTCHA targets."""

import json
import os
import time
from typing import Dict, Optional

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PolygonStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension
from tf2_ros import Buffer, TransformException, TransformListener

from denso_vision.holder_geometry import (
    HolderGeometry, image_tag_order, transform_pixel,
)
from denso_vision.colored_targets import (
    DEFAULT_HSV, TARGET_KINDS, StableTargets, detect_targets, validate_targets,
)
from denso_vision.captcha_targets import CaptchaDetector


class CellphoneHolderDetector(Node):
    """Estimate pixel-to-holder-plane coordinates from tag IDs 1-4."""

    def __init__(self) -> None:
        """Create subscriptions, publishers, detector, and holder geometry."""
        super().__init__('cellphone_holder_detector')

        self.declare_parameter('image_source', 'realsense')
        self.declare_parameter('sim', False)
        self.declare_parameter('target', 'rgb')
        self.declare_parameter('captcha_min_side_mm', 3.0)
        self.declare_parameter('captcha_max_side_mm', 9.0)
        self.declare_parameter('captcha_match_threshold', 0.72)
        self._target_type = self.get_parameter('target').value
        if self._target_type not in ('rgb', 'captcha'):
            raise ValueError('target must be rgb or captcha')
        self._captcha_detector = None
        if self._target_type == 'captcha':
            self._captcha_detector = CaptchaDetector(
                os.path.join(get_package_share_directory('denso_vision'),
                             'assets', 'captcha_checkbox.png'),
                self.get_parameter('captcha_min_side_mm').value,
                self.get_parameter('captcha_max_side_mm').value,
                self.get_parameter('captcha_match_threshold').value,
            )
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
        self.declare_parameter('realsense_auto_exposure', True)
        self.declare_parameter(
            'realsense_auto_exposure_limit_us', 12000.0
        )
        self.declare_parameter('realsense_auto_gain_limit', 32.0)
        self.declare_parameter('realsense_exposure_us', 8000.0)
        self.declare_parameter('realsense_gain', 16.0)
        self.declare_parameter('realsense_screen_exposure_roi', True)
        self.declare_parameter('realsense_exposure_recovery', True)
        self.declare_parameter(
            'realsense_exposure_recovery_us',
            [8000.0, 16000.0, 24000.0, 30000.0, 38000.0, 45000.0],
        )
        self.declare_parameter('realsense_exposure_recovery_delay_sec', 2.0)
        self.declare_parameter('undistort_realsense_color', True)
        self.declare_parameter(
            'holder_frame_id', 'right_cellphone_holder_tags_frame'
        )
        self.declare_parameter('tag_ids', [1, 2, 3, 4])
        self.declare_parameter('phone_orientation', 'vertical')
        self.declare_parameter('vertical_image_tag_order', [1, 2, 4, 3])
        self.declare_parameter('horizontal_image_tag_order', [2, 3, 1, 4])
        self.declare_parameter('tag_size_mm', 21.5)
        self.declare_parameter('top_spacing_mm', 95.2)
        self.declare_parameter('bottom_spacing_mm', 99.8)
        self.declare_parameter('side_spacing_mm', 207.5)
        self.declare_parameter('target_pixel_x', -1.0)
        self.declare_parameter('target_pixel_y', -1.0)
        self.declare_parameter('homography_ransac_threshold_mm', 4.0)
        self.declare_parameter('fallback_binary_threshold', 90)
        self.declare_parameter('tag_clahe_clip_limit', 2.0)
        self.declare_parameter('tag_clahe_grid_size', 8)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_tip_link', 'left_calib_link')
        self.declare_parameter('detect_colored_dots', True)
        self.declare_parameter('min_dot_radius_px', 12.0)
        self.declare_parameter('max_dot_radius_px', 150.0)
        self.declare_parameter('min_dot_circularity', 0.75)
        self.declare_parameter('low_light_value_min', 40)
        self.declare_parameter('low_light_min_dot_circularity', 0.5)
        self.declare_parameter('target_roi_x_min_mm', -50.0)
        self.declare_parameter('target_roi_x_max_mm', 50.0)
        self.declare_parameter('target_roi_y_min_mm', -100.0)
        self.declare_parameter('target_roi_y_max_mm', 85.0)
        self.declare_parameter('stable_frames', 3)
        self.declare_parameter('stable_tolerance_px', 3.0)
        self.declare_parameter('target_diagnostic_dir', '/tmp/denso_target_diagnostics')
        for color, bounds in DEFAULT_HSV.items():
            for index, value in enumerate(bounds):
                self.declare_parameter(f'{color}_hsv_{index}', list(value))
                self.declare_parameter(
                    f'sim_{color}_hsv_{index}', list(value)
                )

        self._roi = tuple(
            self.get_parameter(f'target_roi_{axis}_{side}_mm').value * 0.001
            for axis in ('x', 'y') for side in ('min', 'max')
        )
        if self._roi[0] >= self._roi[1] or self._roi[2] >= self._roi[3]:
            raise ValueError('Target ROI minimum must be less than maximum')
        hsv_prefix = 'sim_' if self.get_parameter('sim').value else ''
        self._hsv_ranges = {
            color: tuple(tuple(self.get_parameter(
                f'{hsv_prefix}{color}_hsv_{index}'
            ).value)
                         for index in range(len(bounds)))
            for color, bounds in DEFAULT_HSV.items()
        }
        self._stable_targets = StableTargets(
            self.get_parameter('stable_frames').value,
            self.get_parameter('stable_tolerance_px').value,
        )
        self._diagnostic_dir = self.get_parameter('target_diagnostic_dir').value
        self._last_diagnostic = 0.0
        self._saved_layout = False
        self._waiting_since = time.monotonic()
        self._last_valid_targets = self._waiting_since
        self._last_exposure_recovery = self._waiting_since
        self._exposure_recovery_index = 0
        self._exposure_recovery_locked = False
        self._using_low_light_detection = False

        self._tag_ids = tuple(
            int(value) for value in self.get_parameter('tag_ids').value
        )
        if len(self._tag_ids) != 4 or len(set(self._tag_ids)) != 4:
            raise ValueError('tag_ids must contain four distinct IDs')
        self._phone_orientation = self.get_parameter(
            'phone_orientation'
        ).value.lower()
        if self._phone_orientation not in ('vertical', 'horizontal'):
            raise ValueError(
                'phone_orientation must be vertical or horizontal'
            )
        order_parameter = f'{self._phone_orientation}_image_tag_order'
        self._expected_image_tag_order = tuple(
            int(value) for value in self.get_parameter(order_parameter).value
        )
        if (
            len(self._expected_image_tag_order) != 4 or
            set(self._expected_image_tag_order) != set(self._tag_ids)
        ):
            raise ValueError(
                f'{order_parameter} must contain each configured tag ID once'
            )

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
        clahe_grid_size = self.get_parameter('tag_clahe_grid_size').value
        if clahe_grid_size < 2:
            raise ValueError('tag_clahe_grid_size must be at least 2')
        self._tag_clahe = cv2.createCLAHE(
            clipLimit=self.get_parameter('tag_clahe_clip_limit').value,
            tileGridSize=(clahe_grid_size, clahe_grid_size),
        )

        self._bridge = CvBridge()
        self._requested_pixel: Optional[np.ndarray] = None
        self._holder_frame_id = self.get_parameter('holder_frame_id').value
        self._debug_tip_link = self.get_parameter('debug_tip_link').value
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._ransac_threshold = (
            self.get_parameter('homography_ransac_threshold_mm').value
            * to_metres
        )
        self._publish_debug = self.get_parameter('publish_debug_image').value
        self._pipeline = None
        self._video_capture = None
        self._video_timer = None
        self._realsense_image_publisher = None
        self._realsense_sensor = None
        self._realsense_roi_sensor = None
        self._realsense_exposure_roi = None
        self._last_exposure_roi_update = 0.0
        self._undistort_map_x = None
        self._undistort_map_y = None
        self._undistortion_profile = None

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
            pipeline_profile = self._pipeline.start(realsense_config)
            self._configure_realsense_exposure(pipeline_profile)
            # Direct RealSense acquisition has no camera-driver node to
            # publish an Image topic. Publish the unannotated BGR image so
            # RViz and other ROS consumers can use image_topic uniformly.
            self._realsense_image_publisher = self.create_publisher(
                Image, image_topic, 10
            )
            self.create_timer(
                1.0 / self.get_parameter('realsense_fps').value,
                self._realsense_callback,
            )
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
        self._targets_publisher = self.create_publisher(
            Float64MultiArray, 'detected_targets', 10
        )

        source = {
            'topic': image_topic,
            'realsense': 'RealSense D405',
            'video': self.get_parameter('video_path').value,
        }[self._image_source]
        self.get_logger().info(
            f'Looking for AprilTag 36h10 IDs {self._tag_ids} on {source}; '
            f'phone={self._phone_orientation}, expected image order='
            f'{self._expected_image_tag_order}'
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
        color_image = self._undistort_realsense_color(
            color_frame, color_image
        )

        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.get_parameter(
            'realsense_frame_id'
        ).value
        image_message = self._bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        image_message.header = message.header
        self._realsense_image_publisher.publish(image_message)
        self._process_image(message, color_image)

    def _configure_realsense_exposure(self, pipeline_profile) -> None:
        """Configure stable exposure for a bright screen in a dark scene."""
        rs = self._rs
        for sensor in pipeline_profile.get_device().query_sensors():
            if sensor.supports(rs.option.enable_auto_exposure):
                self._realsense_sensor = sensor
                break
        if self._realsense_sensor is None:
            self.get_logger().warning(
                'The active RealSense sensor has no exposure control'
            )
            return

        sensor = self._realsense_sensor
        auto = self.get_parameter('realsense_auto_exposure').value
        sensor.set_option(rs.option.enable_auto_exposure, 1.0 if auto else 0.0)
        if auto:
            self._set_realsense_limit(
                'auto_exposure_limit', 'auto_exposure_limit_toggle',
                self.get_parameter(
                    'realsense_auto_exposure_limit_us'
                ).value,
            )
            self._set_realsense_limit(
                'auto_gain_limit', 'auto_gain_limit_toggle',
                self.get_parameter('realsense_auto_gain_limit').value,
            )
            self.get_logger().info(
                'RealSense auto exposure enabled; limits: '
                f"{self.get_parameter('realsense_auto_exposure_limit_us').value:.0f} "
                'us, gain '
                f"{self.get_parameter('realsense_auto_gain_limit').value:.0f}"
            )
        else:
            self._set_realsense_option(
                rs.option.exposure,
                self.get_parameter('realsense_exposure_us').value,
                'realsense_exposure_us',
            )
            self._set_realsense_option(
                rs.option.gain,
                self.get_parameter('realsense_gain').value,
                'realsense_gain',
            )
            self.get_logger().info(
                'RealSense manual exposure: '
                f"{self.get_parameter('realsense_exposure_us').value:.0f} us, "
                f"gain {self.get_parameter('realsense_gain').value:.0f}"
            )

        if auto and self.get_parameter(
            'realsense_screen_exposure_roi'
        ).value:
            try:
                self._realsense_roi_sensor = sensor.as_roi_sensor()
            except RuntimeError as error:
                self.get_logger().warning(
                    f'RealSense exposure ROI is unavailable: {error}'
                )

    def _set_realsense_limit(
        self, option_name: str, toggle_name: str, value: float
    ) -> None:
        """Enable and set an SDK auto-exposure limit when supported."""
        option = getattr(self._rs.option, option_name, None)
        toggle = getattr(self._rs.option, toggle_name, None)
        sensor = self._realsense_sensor
        if option is None or toggle is None:
            return
        if not sensor.supports(option) or not sensor.supports(toggle):
            self.get_logger().warning(
                f'RealSense does not support {option_name}'
            )
            return
        self._set_realsense_option(option, value, option_name)
        sensor.set_option(toggle, 1.0)

    def _set_realsense_option(
        self, option, value: float, parameter_name: str
    ) -> None:
        """Set a numeric SDK option after validating its advertised range."""
        sensor = self._realsense_sensor
        if not sensor.supports(option):
            raise RuntimeError(
                f'RealSense does not support {parameter_name}'
            )
        option_range = sensor.get_option_range(option)
        if not option_range.min <= value <= option_range.max:
            raise ValueError(
                f'{parameter_name}={value} is outside the RealSense range '
                f'[{option_range.min}, {option_range.max}]'
            )
        sensor.set_option(option, float(value))

    def _recover_realsense_exposure(self, targets_valid: bool) -> None:
        """Try deterministic shutter values while the initial layout is lost."""
        if targets_valid:
            self._last_valid_targets = time.monotonic()
            if not self._exposure_recovery_locked:
                self._exposure_recovery_locked = True
                if self._realsense_sensor is not None:
                    exposure = self._realsense_sensor.get_option(
                        self._rs.option.exposure
                    )
                    self.get_logger().info(
                        f'Target exposure locked at {exposure:.0f} us'
                    )
            return
        if (
            self._realsense_sensor is None or
            self._exposure_recovery_locked or
            not self.get_parameter('realsense_exposure_recovery').value
        ):
            return
        now = time.monotonic()
        delay = self.get_parameter(
            'realsense_exposure_recovery_delay_sec'
        ).value
        if (
            now - self._last_valid_targets < delay or
            now - self._last_exposure_recovery < delay
        ):
            return
        candidates = self.get_parameter(
            'realsense_exposure_recovery_us'
        ).value
        if not candidates:
            return
        exposure = float(
            candidates[self._exposure_recovery_index % len(candidates)]
        )
        self._exposure_recovery_index += 1
        sensor = self._realsense_sensor
        sensor.set_option(self._rs.option.enable_auto_exposure, 0.0)
        self._set_realsense_option(
            self._rs.option.exposure, exposure,
            'realsense_exposure_recovery_us',
        )
        self._set_realsense_option(
            self._rs.option.gain,
            self.get_parameter('realsense_gain').value,
            'realsense_gain',
        )
        self._stable_targets.update([])
        self._last_exposure_recovery = now
        self.get_logger().warning(
            f'No valid layout; trying RealSense exposure {exposure:.0f} us'
        )

    def _detect_rgb_targets(
        self, image: np.ndarray, homography: np.ndarray
    ):
        """Detect RGB targets, with a validated low-light fallback."""
        parameters = (
            self.get_parameter('min_dot_radius_px').value,
            self.get_parameter('max_dot_radius_px').value,
        )
        primary = detect_targets(
            image, homography, self._roi, *parameters,
            self._hsv_ranges,
            self.get_parameter('min_dot_circularity').value,
        )
        try:
            validate_targets(primary, 'rgb')
        except ValueError:
            value_min = self.get_parameter('low_light_value_min').value
            low_light_ranges = {}
            for color, bounds in self._hsv_ranges.items():
                adjusted = []
                for index, bound in enumerate(bounds):
                    if index % 2 == 0:
                        adjusted.append((bound[0], bound[1], value_min))
                    else:
                        adjusted.append(tuple(bound))
                low_light_ranges[color] = tuple(adjusted)
            fallback = detect_targets(
                image, homography, self._roi, *parameters,
                low_light_ranges,
                self.get_parameter(
                    'low_light_min_dot_circularity'
                ).value,
            )
            try:
                validate_targets(fallback, 'rgb')
            except ValueError:
                return max((primary, fallback), key=len), False
            if not self._using_low_light_detection:
                self.get_logger().info(
                    'Using validated low-light RGB detection'
                )
            self._using_low_light_detection = True
            return fallback, True
        self._using_low_light_detection = False
        return primary, True

    def _update_realsense_exposure_roi(
        self, homography: np.ndarray, image_shape: tuple
    ) -> None:
        """Meter auto exposure on the phone screen derived from homography."""
        if self._realsense_roi_sensor is None:
            return
        try:
            x_min, x_max, y_min, y_max = self._roi
            holder_corners = np.array([[
                [x_min, y_min], [x_max, y_min],
                [x_max, y_max], [x_min, y_max],
            ]], dtype=np.float64)
            image_corners = cv2.perspectiveTransform(
                holder_corners, np.linalg.inv(homography)
            )[0]
        except (cv2.error, np.linalg.LinAlgError):
            return
        if not np.all(np.isfinite(image_corners)):
            return

        height, width = image_shape[:2]
        minimum = np.floor(image_corners.min(axis=0)).astype(int)
        maximum = np.ceil(image_corners.max(axis=0)).astype(int)
        roi = (
            int(np.clip(minimum[0], 0, width - 2)),
            int(np.clip(minimum[1], 0, height - 2)),
            int(np.clip(maximum[0], 1, width - 1)),
            int(np.clip(maximum[1], 1, height - 1)),
        )
        if roi[2] - roi[0] < 16 or roi[3] - roi[1] < 16:
            return
        now = time.monotonic()
        if self._realsense_exposure_roi is not None:
            movement = max(
                abs(current - previous)
                for current, previous in zip(
                    roi, self._realsense_exposure_roi
                )
            )
            if movement < 12 or now - self._last_exposure_roi_update < 0.5:
                return

        sdk_roi = self._rs.region_of_interest()
        sdk_roi.min_x, sdk_roi.min_y, sdk_roi.max_x, sdk_roi.max_y = roi
        try:
            self._realsense_roi_sensor.set_region_of_interest(sdk_roi)
        except RuntimeError as error:
            self.get_logger().warning(
                f'Could not update RealSense exposure ROI: {error}'
            )
            self._realsense_roi_sensor = None
            return
        self._realsense_exposure_roi = roi
        self._last_exposure_roi_update = now
        self.get_logger().info(
            f'RealSense exposure ROI set to screen pixels {roi}'
        )

    def _undistort_realsense_color(
        self, color_frame, image: np.ndarray
    ) -> np.ndarray:
        """Return a rectified D405 colour image using its active SDK profile.

        RealSense returns the camera's active intrinsics with every stream
        profile.  A profile whose model is ``none`` (or whose coefficients are
        zero) is already rectified.  Otherwise, create a destination-to-source
        map once: each ideal pixel is converted into a unit-depth ray and the
        RealSense SDK projects that ray into the distorted source frame.  This
        covers every distortion model exposed by librealsense, including its
        inverse Brown-Conrady model, which OpenCV cannot represent directly.
        """
        if not self.get_parameter('undistort_realsense_color').value:
            return image
        intrinsics = color_frame.profile.as_video_stream_profile().get_intrinsics()
        signature = (
            intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy,
            intrinsics.ppx, intrinsics.ppy, int(intrinsics.model),
            tuple(float(value) for value in intrinsics.coeffs),
        )
        if signature != self._undistortion_profile:
            self._undistortion_profile = signature
            self._undistort_map_x, self._undistort_map_y = \
                self._create_realsense_undistort_maps(intrinsics)
        if self._undistort_map_x is None:
            return image
        return cv2.remap(
            image, self._undistort_map_x, self._undistort_map_y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )

    def _create_realsense_undistort_maps(self, intrinsics):
        """Create rectified-pixel to raw-pixel maps from SDK intrinsics."""
        coefficients = np.asarray(intrinsics.coeffs, dtype=np.float64)
        if (
            intrinsics.model == self._rs.distortion.none or
            np.allclose(coefficients, 0.0)
        ):
            self.get_logger().info(
                'RealSense colour profile is already rectified: '
                f'{intrinsics.width}x{intrinsics.height}, '
                f'model={intrinsics.model}'
            )
            return None, None

        height, width = intrinsics.height, intrinsics.width
        map_x = np.empty((height, width), dtype=np.float32)
        map_y = np.empty((height, width), dtype=np.float32)
        try:
            normalized_x = (np.arange(width, dtype=np.float64) - intrinsics.ppx) / intrinsics.fx
            for row in range(height):
                normalized_y = (row - intrinsics.ppy) / intrinsics.fy
                for column, x_coordinate in enumerate(normalized_x):
                    source = self._rs.rs2_project_point_to_pixel(
                        intrinsics, [float(x_coordinate), float(normalized_y), 1.0]
                    )
                    map_x[row, column] = source[0]
                    map_y[row, column] = source[1]
        except (AttributeError, RuntimeError) as error:
            # Standard OpenCV Brown-Conrady fallback for older bindings that
            # do not expose the librealsense projection helper.
            self.get_logger().warning(
                f'RealSense SDK projection unavailable ({error}); using OpenCV undistortion'
            )
            camera_matrix = np.array([
                [intrinsics.fx, 0.0, intrinsics.ppx],
                [0.0, intrinsics.fy, intrinsics.ppy],
                [0.0, 0.0, 1.0],
            ])
            map_x, map_y = cv2.initUndistortRectifyMap(
                camera_matrix, coefficients, None, camera_matrix,
                (width, height), cv2.CV_32FC1,
            )
        self.get_logger().info(
            'Rectifying RealSense colour frames with active SDK intrinsics: '
            f'{width}x{height}, model={intrinsics.model}, coeffs={coefficients.tolist()}'
        )
        return map_x, map_y

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

    def _image_tag_order(
        self, corners: list, ids: Optional[np.ndarray]
    ) -> Optional[tuple]:
        """Return the observed TL, TR, BL, BR order when all tags exist."""
        if ids is None:
            return None
        centres: Dict[int, np.ndarray] = {}
        for tag_corners, tag_id in zip(corners, ids.flatten()):
            numeric_id = int(tag_id)
            if numeric_id in self._tag_ids and numeric_id not in centres:
                centres[numeric_id] = np.asarray(
                    tag_corners, dtype=np.float64
                ).reshape(4, 2).mean(axis=0)
        if len(centres) != 4:
            return None
        return image_tag_order(centres)

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

    def _draw_tip_projection(
        self, image: np.ndarray, homography: np.ndarray
    ) -> Optional[str]:
        """Draw the current calibration-tip origin projected onto the image.

        The fitted homography maps image pixels to the holder plane.  The
        inverse maps the tip's current holder-plane X/Y location back to the
        D405 image, which makes an alignment error immediately visible in the
        same view used to inspect the tags and RGB targets.
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                self._holder_frame_id, self._debug_tip_link, Time()
            )
            tip = transform.transform.translation
            inverse = np.linalg.inv(homography)
            projected = cv2.perspectiveTransform(
                np.array([[[tip.x, tip.y]]], dtype=np.float64), inverse
            )[0, 0]
        except (TransformException, np.linalg.LinAlgError) as error:
            self.get_logger().debug(
                f'Cannot project {self._debug_tip_link}: {error}'
            )
            return None

        if not np.all(np.isfinite(projected)):
            return f'Tip projection is not finite ({tip.x:.3f}, {tip.y:.3f}) m'
        pixel = tuple(np.rint(projected).astype(int))
        in_image = 0 <= pixel[0] < image.shape[1] and 0 <= pixel[1] < image.shape[0]
        if in_image:
            # Magenta dot: distinct from the red requested-pixel cross and RGB targets.
            cv2.circle(image, pixel, 7, (255, 0, 255), -1, cv2.LINE_AA)
            cv2.circle(image, pixel, 2, (0, 0, 0), -1, cv2.LINE_AA)
            return f'Tip px: ({pixel[0]}, {pixel[1]})'
        return f'Tip px outside image: ({pixel[0]}, {pixel[1]})'

    @staticmethod
    def _draw_detected_blobs(image: np.ndarray, targets) -> None:
        """Draw every target and its centre on the debug image."""
        colors = {
            'red': (0, 0, 255),
            'green': (0, 220, 0),
            'blue': (255, 0, 0),
            'captcha': (0, 180, 255),
        }
        counts = {color: 0 for color in colors}
        for target in targets:
            counts[target.color] += 1
            color = colors[target.color]
            center = (int(round(target.pixel_x)), int(round(target.pixel_y)))
            radius = max(1, int(round(target.radius_px)))
            if target.color == 'captcha':
                cv2.rectangle(image, (center[0] - radius, center[1] - radius),
                              (center[0] + radius, center[1] + radius), color, 2)
            else:
                cv2.circle(image, center, radius, color, 2, cv2.LINE_AA)
            cv2.drawMarker(
                image, center, color, cv2.MARKER_CROSS, 12, 2, cv2.LINE_AA
            )
            label = (
                f'{target.color[0].upper()}{counts[target.color]} '
                f'({center[0]}, {center[1]}) r={radius}'
            )
            label_position = (center[0] + radius + 4, center[1] - radius - 4)
            cv2.putText(
                image, label, label_position, cv2.FONT_HERSHEY_SIMPLEX,
                0.45, (0, 0, 0), 3, cv2.LINE_AA,
            )
            cv2.putText(
                image, label, label_position, cv2.FONT_HERSHEY_SIMPLEX,
                0.45, color, 1, cv2.LINE_AA,
            )

    def _image_callback(self, message: Image) -> None:
        try:
            color_image = self._bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8'
            )
        except CvBridgeError as error:
            self.get_logger().error(f'Could not convert image: {error}')
            return

        self._process_image(message, color_image)

    def _save_diagnostic(self, image: np.ndarray, reason: str) -> None:
        if time.monotonic() - self._last_diagnostic < 5.0:
            return
        self._last_diagnostic = time.monotonic()
        os.makedirs(self._diagnostic_dir, exist_ok=True)
        path = os.path.join(self._diagnostic_dir, f'targets_{time.time_ns()}.png')
        if cv2.imwrite(path, image):
            self.get_logger().warning(f'{reason}; screenshot: {path}')
        else:
            self.get_logger().error(f'{reason}; could not save screenshot: {path}')

    def _process_image(
        self, message: Image, color_image: np.ndarray
    ) -> None:
        """Detect tags and publish homography results for one BGR image."""
        raw_image = color_image.copy()
        targets_processed = False
        targets_valid = False
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray_image)
        ordered_points = self._ordered_detections(corners, ids)
        observed_order = self._image_tag_order(corners, ids)

        # A phone exposed correctly can leave the surrounding paper tags very
        # dark. Keep the raw image first, then recover local tag contrast with
        # CLAHE. The binary fallback remains useful for Gazebo's lit textures.
        if ordered_points is None and observed_order is None:
            enhanced_image = self._tag_clahe.apply(gray_image)
            enhanced_corners, enhanced_ids, _ = self._detect(enhanced_image)
            enhanced_points = self._ordered_detections(
                enhanced_corners, enhanced_ids
            )
            if enhanced_points is not None:
                corners, ids, ordered_points = (
                    enhanced_corners,
                    enhanced_ids,
                    enhanced_points,
                )
                observed_order = self._image_tag_order(corners, ids)

        if ordered_points is None and observed_order is None:
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
                observed_order = self._image_tag_order(corners, ids)

        detected_tag_ids = set() if ids is None else set(ids.flatten())
        overlay_lines = [
            f'Tags: {len(detected_tag_ids & set(self._tag_ids))}/4',
            f'Phone: {self._phone_orientation}',
        ]
        if observed_order is not None:
            observed_text = ' '.join(str(value) for value in observed_order)
            expected_text = ' '.join(
                str(value) for value in self._expected_image_tag_order
            )
            overlay_lines.append(
                f'Tag order: {observed_text} (expected {expected_text})'
            )
            if observed_order != self._expected_image_tag_order:
                ordered_points = None
                self._save_diagnostic(
                    raw_image,
                    f'Tag order {observed_order} does not match '
                    f'{self._phone_orientation} mode '
                    f'{self._expected_image_tag_order}',
                )

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
                self._update_realsense_exposure_roi(
                    homography, color_image.shape
                )
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
                tip_overlay = self._draw_tip_projection(color_image, homography)
                if tip_overlay:
                    overlay_lines.append(tip_overlay)
                if self._target_type == 'captcha' or self.get_parameter('detect_colored_dots').value:
                    targets_processed = True
                    if self._target_type == 'captcha':
                        targets = self._captcha_detector.detect(raw_image, homography, self._roi)
                        try:
                            validate_targets(targets, self._target_type)
                            targets_valid = True
                        except ValueError:
                            pass
                    else:
                        targets, targets_valid = self._detect_rgb_targets(
                            raw_image, homography
                        )
                    self._draw_detected_blobs(color_image, targets)
                    blob_counts = {
                        color: sum(target.color == color for target in targets)
                        for color in ('red', 'green', 'blue')
                    }
                    overlay_lines.append(
                        f'Checkboxes: {len(targets)}' if self._target_type == 'captcha' else
                        'Blobs: {} (R:{} G:{} B:{})'.format(
                            len(targets), blob_counts['red'],
                            blob_counts['green'], blob_counts['blue'],
                        )
                    )
                    if self._stable_targets.update(targets):
                        try:
                            validate_targets(targets, self._target_type)
                        except ValueError as error:
                            self._save_diagnostic(raw_image, str(error))
                        else:
                            if not self._saved_layout:
                                os.makedirs(self._diagnostic_dir, exist_ok=True)
                                stem = os.path.join(
                                    self._diagnostic_dir,
                                    f'layout_{time.time_ns()}',
                                )
                                if not cv2.imwrite(stem + '.png', raw_image):
                                    self.get_logger().error(
                                        'Could not save target layout: '
                                        f'{stem}.png'
                                    )
                                    return
                                with open(stem + '.json', 'w', encoding='utf-8') as output:
                                    json.dump(
                                        [target.__dict__ for target in targets],
                                        output,
                                        indent=2,
                                    )
                                self.get_logger().info(
                                    'Saved stable target layout: '
                                    f'{stem}.png/.json'
                                )
                                self._saved_layout = True
                            data = []
                            for target in targets:
                                data.extend((
                                    float(TARGET_KINDS.index(target.color)),
                                    target.pixel_x, target.pixel_y, target.radius_px,
                                    target.holder_x, target.holder_y,
                                ))
                            target_message = Float64MultiArray(data=data)
                            target_message.layout.dim = [MultiArrayDimension(
                                label=self._holder_frame_id,
                                size=len(targets), stride=len(data),
                            )]
                            self._targets_publisher.publish(target_message)
                            self._waiting_since = time.monotonic()
                            for color in ('red', 'green', 'blue'):
                                target = next((item for item in targets if item.color == color), None)
                                if target is None:
                                    continue
                                point = PointStamped()
                                point.header = message.header
                                point.header.frame_id = self._holder_frame_id
                                point.point.x = target.holder_x
                                point.point.y = target.holder_y
                                self._dot_publishers[color].publish(point)
                            overlay_lines.append(f'Targets: {len(targets)} stable')
                    elif targets:
                        overlay_lines.append(f'Targets: {len(targets)} stabilizing')
                cv2.drawMarker(
                    color_image,
                    tuple(np.rint(target_pixel).astype(int)),
                    (0, 0, 255),
                    cv2.MARKER_CROSS,
                    24,
                    2,
                )

        if not targets_processed:
            self._stable_targets.update([])
        self._recover_realsense_exposure(targets_valid)
        if not self._saved_layout and time.monotonic() - self._waiting_since >= 5.0:
            self._save_diagnostic(
                raw_image, f'Waiting for stable {self._target_type} targets and four tags'
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
