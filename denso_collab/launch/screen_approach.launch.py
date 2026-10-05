"""Run a bounded number of random-holder and screen-approach tests."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _stop(reason):
    return [EmitEvent(event=Shutdown(reason=reason))]


def _start_tests(context):
    try:
        runs = int(LaunchConfiguration('test_runs').perform(context))
        if not 1 <= runs <= 100:
            raise ValueError('test_runs must be between 1 and 100')
        random_enabled = LaunchConfiguration('use_random_holder_pose').perform(context).lower()
        if random_enabled not in ('true', 'false'):
            raise ValueError('use_random_holder_pose must be true or false')
        sim = LaunchConfiguration('sim').perform(context).lower()
        touch_enabled = LaunchConfiguration('touch_enabled').perform(context).lower()
        if sim not in ('true', 'false') or touch_enabled not in ('true', 'false'):
            raise ValueError('sim and touch_enabled must be true or false')
    except ValueError as error:
        return _stop(str(error))

    collab_config = get_package_share_directory('denso_collab') + '/config/cellphone_collab.yaml'
    vision_launch = PythonLaunchDescriptionSource([
        get_package_share_directory('denso_vision'),
        '/launch/cellphone_holder_vision.launch.py',
    ])
    vision_started = False

    def screen_stage():
        nonlocal vision_started
        actions = []
        if not vision_started:
            actions.append(IncludeLaunchDescription(vision_launch, launch_arguments={
                'image_source': LaunchConfiguration('image_source'),
                'image_topic': LaunchConfiguration('image_topic'),
                'video_path': LaunchConfiguration('video_path'),
            }.items()))
            if sim == 'true' and touch_enabled == 'true':
                actions.append(Node(
                    package='denso_collab', executable='sim_touch_contact.py',
                    name='sim_touch_contact', output='screen',
                    parameters=[collab_config],
                ))
            vision_started = True
        screen = Node(
            package='denso_collab',
            executable='move_to_screen_standoff',
            name='move_to_screen_standoff',
            output='screen',
            parameters=[collab_config, {
                'plan_only': ParameterValue(
                    LaunchConfiguration('approach_plan_only'), value_type=bool),
                'touch_enabled': ParameterValue(
                    LaunchConfiguration('touch_enabled'), value_type=bool),
                'touch_disable_collision_check': ParameterValue(
                    LaunchConfiguration('touch_disable_collision_check'), value_type=bool),
                'max_targets': ParameterValue(
                    LaunchConfiguration('max_targets'), value_type=int),
                'sim': ParameterValue(LaunchConfiguration('sim'), value_type=bool),
            }],
        )
        actions.extend([screen, RegisterEventHandler(OnProcessExit(
            target_action=screen, on_exit=after_screen))])
        return actions

    def random_stage():
        random_pose = Node(
            package='denso_collab', executable='random_collab_pose', output='screen')
        return [random_pose, RegisterEventHandler(OnProcessExit(
            target_action=random_pose, on_exit=after_random))]

    def after_random(event, _context):
        if event.returncode != 0:
            return _stop('random_collab_pose failed; screen approach skipped')
        return screen_stage()

    def after_screen(event, _context):
        nonlocal runs
        if event.returncode != 0:
            return _stop('screen approach failed; further tests skipped')
        runs -= 1
        if runs == 0:
            return _stop('All test runs completed')
        return random_stage() if random_enabled == 'true' else screen_stage()

    return random_stage() if random_enabled == 'true' else screen_stage()


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('test_runs', default_value='1'),
        DeclareLaunchArgument('use_random_holder_pose', default_value='false'),
        DeclareLaunchArgument('image_source', default_value='realsense'),
        DeclareLaunchArgument('image_topic', default_value='/basic_camera'),
        DeclareLaunchArgument('video_path', default_value=''),
        DeclareLaunchArgument('approach_plan_only', default_value='false'),
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('touch_enabled', default_value='false'),
        DeclareLaunchArgument('touch_disable_collision_check', default_value='false'),
        DeclareLaunchArgument('max_targets', default_value='0'),
        OpaqueFunction(function=_start_tests),
    ])
