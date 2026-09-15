"""Launch the cellphone-holder AprilTag detector."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('image_topic', default_value='/basic_camera'),
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
                'sim': ParameterValue(
                    LaunchConfiguration('sim'), value_type=bool
                ),
                'image_topic': LaunchConfiguration('image_topic'),
                'realsense_serial': LaunchConfiguration('realsense_serial'),
            }],
        ),
    ])
