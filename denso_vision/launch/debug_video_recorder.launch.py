"""Record the detector debug image as a compact MP4 video."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the debug video recorder launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('image_topic', default_value='debug_image'),
        DeclareLaunchArgument(
            'output_dir',
            default_value=(
                '/root/denso_ws/src/denso_robot_ros2/debug_videos'
            ),
        ),
        DeclareLaunchArgument('filename_prefix', default_value='debug_image'),
        DeclareLaunchArgument('record_fps', default_value='15.0'),
        DeclareLaunchArgument('scale', default_value='1.0'),
        DeclareLaunchArgument('codec', default_value='avc1'),
        Node(
            package='denso_vision',
            executable='debug_video_recorder',
            name='debug_video_recorder',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'output_dir': LaunchConfiguration('output_dir'),
                'filename_prefix': LaunchConfiguration('filename_prefix'),
                'record_fps': ParameterValue(
                    LaunchConfiguration('record_fps'), value_type=float
                ),
                'scale': ParameterValue(
                    LaunchConfiguration('scale'), value_type=float
                ),
                'codec': LaunchConfiguration('codec'),
            }],
        ),
    ])
