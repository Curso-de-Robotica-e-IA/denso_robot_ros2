"""Launch the cellphone-holder AprilTag detector."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
        DeclareLaunchArgument('video_path', default_value=''),
        DeclareLaunchArgument('video_loop', default_value='true'),
        DeclareLaunchArgument('video_fps', default_value='0.0'),
        DeclareLaunchArgument(
            'realsense_serial',
            default_value=EnvironmentVariable(
                'REALSENSE_SERIAL', default_value=''
            ),
        ),
        Node(
            package='denso_vision',
            executable='cellphone_holder_detector',
            name='cellphone_holder_detector',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[config, {
                'image_source': LaunchConfiguration('image_source'),
                'image_topic': LaunchConfiguration('image_topic'),
                'video_path': LaunchConfiguration('video_path'),
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
            }],
        ),
    ])
