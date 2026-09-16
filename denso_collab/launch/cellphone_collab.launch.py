"""Run cellphone perception and the red-green-blue coarse approach."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vision_launch = PythonLaunchDescriptionSource([
        get_package_share_directory('denso_vision'),
        '/launch/cellphone_holder_vision.launch.py',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('image_source', default_value='realsense'),
        DeclareLaunchArgument('image_topic', default_value='/basic_camera'),
        DeclareLaunchArgument('video_path', default_value=''),
        DeclareLaunchArgument('standoff_m', default_value='0.1'),
        DeclareLaunchArgument('plan_only', default_value='false'),
        IncludeLaunchDescription(vision_launch, launch_arguments={
            'image_source': LaunchConfiguration('image_source'),
            'image_topic': LaunchConfiguration('image_topic'),
            'video_path': LaunchConfiguration('video_path'),
        }.items()),
        Node(
            package='denso_collab',
            executable='move_to_screen_standoff',
            name='move_to_screen_standoff',
            output='screen',
            parameters=[{
                'standoff_m': LaunchConfiguration('standoff_m'),
                'plan_only': LaunchConfiguration('plan_only'),
            }],
        ),
    ])
