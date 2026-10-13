"""Launch the cellphone-holder AprilTag detector."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the detector launch description."""
    config = os.path.join(
        get_package_share_directory('denso_vision'),
        'config',
        'cellphone_holder.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'image_source', default_value='realsense',
            description='Image input: video, topic, or realsense',
        ),
        DeclareLaunchArgument('image_topic', default_value='/basic_camera'),
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('target', default_value='rgb', choices=['rgb', 'captcha']),
        DeclareLaunchArgument(
            'phone_orientation', default_value='vertical',
            description='Phone pose: vertical or horizontal (+90 deg CCW)',
        ),
        DeclareLaunchArgument('video_path', default_value=''),
        DeclareLaunchArgument('video_loop', default_value='true'),
        DeclareLaunchArgument('video_fps', default_value='0.0'),
        DeclareLaunchArgument('record_debug_video', default_value='false'),
        DeclareLaunchArgument('debug_video_fps', default_value='15.0'),
        DeclareLaunchArgument('debug_video_scale', default_value='1.0'),
        DeclareLaunchArgument(
            'debug_video_output_dir',
            default_value=(
                '/root/denso_ws/src/denso_robot_ros2/debug_videos'
            ),
        ),
        DeclareLaunchArgument(
            'realsense_serial',
            default_value=EnvironmentVariable(
                'REALSENSE_SERIAL', default_value=''
            ),
        ),
        DeclareLaunchArgument(
            'realsense_auto_exposure', default_value='true'
        ),
        DeclareLaunchArgument(
            'realsense_auto_exposure_limit_us', default_value='12000.0'
        ),
        DeclareLaunchArgument(
            'realsense_auto_gain_limit', default_value='32.0'
        ),
        DeclareLaunchArgument(
            'realsense_screen_exposure_roi', default_value='true'
        ),
        DeclareLaunchArgument(
            'realsense_exposure_us', default_value='8000.0'
        ),
        DeclareLaunchArgument('realsense_gain', default_value='16.0'),
        Node(
            package='denso_vision',
            executable='cellphone_holder_detector',
            name='cellphone_holder_detector',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[config, {
                'image_source': LaunchConfiguration('image_source'),
                'target': LaunchConfiguration('target'),
                'sim': ParameterValue(
                    LaunchConfiguration('sim'), value_type=bool
                ),
                'image_topic': LaunchConfiguration('image_topic'),
                'video_path': LaunchConfiguration('video_path'),
                'phone_orientation': ParameterValue(
                    LaunchConfiguration('phone_orientation'), value_type=str
                ),
                'video_loop': ParameterValue(
                    LaunchConfiguration('video_loop'), value_type=bool
                ),
                'video_fps': ParameterValue(
                    LaunchConfiguration('video_fps'), value_type=float
                ),
                # Serial numbers containing only digits would otherwise be
                # inferred as integers by ROS parameter parsing.
                'realsense_serial': ParameterValue(
                    LaunchConfiguration('realsense_serial'), value_type=str
                ),
                'realsense_auto_exposure': ParameterValue(
                    LaunchConfiguration('realsense_auto_exposure'),
                    value_type=bool,
                ),
                'realsense_auto_exposure_limit_us': ParameterValue(
                    LaunchConfiguration('realsense_auto_exposure_limit_us'),
                    value_type=float,
                ),
                'realsense_auto_gain_limit': ParameterValue(
                    LaunchConfiguration('realsense_auto_gain_limit'),
                    value_type=float,
                ),
                'realsense_screen_exposure_roi': ParameterValue(
                    LaunchConfiguration('realsense_screen_exposure_roi'),
                    value_type=bool,
                ),
                'realsense_exposure_us': ParameterValue(
                    LaunchConfiguration('realsense_exposure_us'),
                    value_type=float,
                ),
                'realsense_gain': ParameterValue(
                    LaunchConfiguration('realsense_gain'), value_type=float
                ),
            }],
        ),
        Node(
            package='denso_vision',
            executable='debug_video_recorder',
            name='debug_video_recorder',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            condition=IfCondition(LaunchConfiguration('record_debug_video')),
            parameters=[{
                'image_topic': 'debug_image',
                'output_dir': LaunchConfiguration(
                    'debug_video_output_dir'
                ),
                'record_fps': ParameterValue(
                    LaunchConfiguration('debug_video_fps'), value_type=float
                ),
                'scale': ParameterValue(
                    LaunchConfiguration('debug_video_scale'), value_type=float
                ),
            }],
        ),
    ])
