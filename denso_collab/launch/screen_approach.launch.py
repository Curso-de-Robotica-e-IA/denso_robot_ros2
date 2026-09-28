"""Detect screen targets and approach them, optionally after a random holder pose."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def screen_actions():
    collab_config = (
        get_package_share_directory('denso_collab')
        + '/config/cellphone_collab.yaml'
    )
    vision_launch = PythonLaunchDescriptionSource([
        get_package_share_directory('denso_vision'),
        '/launch/cellphone_holder_vision.launch.py',
    ])
    return [
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
            parameters=[collab_config, {
                'plan_only': ParameterValue(
                    LaunchConfiguration('approach_plan_only'), value_type=bool),
            }],
        ),
    ]


def after_random(event, _context):
    if event.returncode != 0:
        return [EmitEvent(event=Shutdown(
            reason='random_collab_pose failed; screen approach skipped',
        ))]
    return screen_actions()


def generate_launch_description():
    use_random = LaunchConfiguration('use_random_holder_pose')
    random_pose = Node(
        package='denso_collab',
        executable='random_collab_pose',
        output='screen',
        condition=IfCondition(use_random),
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_random_holder_pose', default_value='false'),
        DeclareLaunchArgument('image_source', default_value='realsense'),
        DeclareLaunchArgument('image_topic', default_value='/basic_camera'),
        DeclareLaunchArgument('video_path', default_value=''),
        DeclareLaunchArgument('approach_plan_only', default_value='false'),
        random_pose,
        GroupAction(actions=screen_actions(), condition=UnlessCondition(use_random)),
        RegisterEventHandler(OnProcessExit(
            target_action=random_pose,
            on_exit=after_random,
        )),
    ])
