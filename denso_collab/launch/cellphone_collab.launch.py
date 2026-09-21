"""Run cellphone perception and the red-green-blue coarse approach."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    vision_launch = PythonLaunchDescriptionSource([
        get_package_share_directory('denso_vision'),
        '/launch/cellphone_holder_vision.launch.py',
    ])
    image_source = LaunchConfiguration('image_source')
    image_topic = LaunchConfiguration('image_topic')
    video_path = LaunchConfiguration('video_path')
    standoff_m = LaunchConfiguration('standoff_m')
    plan_only = LaunchConfiguration('plan_only')
    sim = LaunchConfiguration('sim')

    start_pose = Node(
        package='denso_collab',
        executable='move_to_collab_start',
        name='move_to_collab_start',
        output='screen',
        parameters=[{
            'sim': ParameterValue(sim, value_type=bool),
            'plan_only': ParameterValue(plan_only, value_type=bool),
        }],
    )
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
            parameters=[{
                'standoff_m': standoff_m,
                'plan_only': ParameterValue(plan_only, value_type=bool),
            }],
        ),
    ]

    def start_vision_after_pose(event, _context):
        if event.returncode != 0:
            return [LogInfo(
                msg=(
                    'Collaboration launch stopped: moving to the initial '
                    'pose failed.'
                )
            )]
        return vision_and_screen

    return LaunchDescription([
        DeclareLaunchArgument('image_source', default_value='realsense'),
        DeclareLaunchArgument('image_topic', default_value='/basic_camera'),
        DeclareLaunchArgument('video_path', default_value=''),
        DeclareLaunchArgument('standoff_m', default_value='0.1'),
        DeclareLaunchArgument('plan_only', default_value='false'),
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='Use the saved Gazebo pose instead of the real pose.',
        ),
        start_pose,
        RegisterEventHandler(OnProcessExit(
            target_action=start_pose,
            on_exit=start_vision_after_pose,
        )),
    ])
