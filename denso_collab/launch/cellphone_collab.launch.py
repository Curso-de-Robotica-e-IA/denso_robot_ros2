"""Run cellphone perception and the red-green-blue coarse approach."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    collab_config = (
        get_package_share_directory('denso_collab')
        + '/config/cellphone_collab.yaml'
    )
    vision_launch = PythonLaunchDescriptionSource([
        get_package_share_directory('denso_vision'),
        '/launch/cellphone_holder_vision.launch.py',
    ])
    image_source = LaunchConfiguration('image_source')
    image_topic = LaunchConfiguration('image_topic')
    video_path = LaunchConfiguration('video_path')
    standoff_m = LaunchConfiguration('standoff_m')
    plan_only = LaunchConfiguration('plan_only')
    vision_and_screen = [
        IncludeLaunchDescription(vision_launch, launch_arguments={
            'image_source': image_source,
            'image_topic': image_topic,
            'video_path': video_path,
        }.items()),
        Node(
            package='denso_collab',
            executable='move_to_screen_standoff',
            name='move_to_screen_standoff',
            output='screen',
            parameters=[collab_config, {
                'standoff_m': standoff_m,
                'plan_only': ParameterValue(plan_only, value_type=bool),
            }],
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('image_source', default_value='realsense'),
        DeclareLaunchArgument('image_topic', default_value='/basic_camera'),
        DeclareLaunchArgument('video_path', default_value=''),
        DeclareLaunchArgument('standoff_m', default_value='0.1'),
        DeclareLaunchArgument('plan_only', default_value='false'),
        *vision_and_screen,
    ])
