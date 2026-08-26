# Copyright (c) 2021 DENSO WAVE INCORPORATED
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: DENSO WAVE INCORPORATED


import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from typing import Text
from launch.launch_context import LaunchContext
from launch.substitution import Substitution
from typing import Iterable
from typing import Text
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.launch_description_sources import PythonLaunchDescriptionSource


""" Function for loading a yaml file. """


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


""" Substitution class for appending LaunchConfig parameters to a string.

Helpful for namespaces and/or MULTI-ROBOT applications.
"""


class TextJoinSubstitution(Substitution):
    """Substitution that join paths, in a platform independent way."""

    def __init__(
            self, substitutions: Iterable[SomeSubstitutionsType], text: Text,
            sequence: Text) -> None:
        super().__init__()
        """Create a TextJoinSubstitution."""
        from launch.utilities import normalize_to_list_of_substitutions
        self.__substitutions = normalize_to_list_of_substitutions(substitutions)
        self.__text = text
        self.__sequence = sequence

    @property
    def substitutions(self) -> Iterable[Substitution]:
        """Getter for variable_name."""
        return self.__substitutions

    def text(self) -> Text:
        """Getter for text."""
        return self.__text

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return "LocalVar('{}')".format(' + '.join([s.describe() for s in self.substitutions]))

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by retrieving the local variable."""
        performed_substitutions = [sub.perform(context) for sub in self.__substitutions]
        return self.__sequence.join(performed_substitutions) + self.__sequence + self.__text


""" Launch Description generator function. """


def generate_launch_description():

    declared_arguments = []

# Denso specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'model',
            choices=['vs050'],
            description='Type/series of used denso robot.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'send_format', default_value='288',
            description='Data format for sending commands to the robot.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'recv_format', default_value='292',
            description='Data format for receiving robot status.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'bcap_slave_control_cycle_msec', default_value='8.0',
            description='Control frequency.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'left_ip_address', default_value='192.168.0.1',
            description='IP address by which the left robot can be reached.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'right_ip_address', default_value='192.168.0.2',
            description='IP address by which the right robot can be reached.'))
# Configuration arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package', default_value='denso_robot_descriptions',
            description='Description package with robot URDF/XACRO files. Usually the argument' \
                + ' is not set, it enables use of a custom description.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file', default_value='dual_denso_robot.urdf.xacro',
            description='URDF/XACRO description file with the robot.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'moveit_config_package', default_value='denso_robot_moveit_config',
            description='MoveIt config package with robot SRDF/XACRO files. Usually the argument' \
                + ' is not set, it enables use of a custom moveit config.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'moveit_config_file', default_value='dual_denso_robot.srdf.xacro',
            description='MoveIt SRDF/XACRO description file with the robot.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace', default_value='',
            description="Prefix of the joint names, useful for" \
                + " multi-robot setup. If changed than also joint names in the controllers'" \
                + " configuration have to be updated."))
    declared_arguments.append(
        DeclareLaunchArgument(
            'controllers_file', default_value='denso_robot_controllers.yaml',
            description='YAML file with the controllers configuration.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'left_robot_controller', default_value='left_denso_joint_trajectory_controller',
            description='Left robot controller to start.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'right_robot_controller', default_value='right_denso_joint_trajectory_controller',
            description='Right robot controller to start.'))
    declared_arguments.append(
        DeclareLaunchArgument('rviz', default_value='false', description='Launch RViz?')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'sim', default_value='true',
            description='Start robot with fake hardware mirroring command to its states.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'verbose', default_value='false',
            description='Print out additional debug information.'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'basic_camera', default_value='false',
            description='Add basic_camera in J6'
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            'left_xyz', default_value='-0.417 0 0',
            description='XYZ position of left arm'
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            'left_rpy', default_value='0 0 0',
            description='RPY position of left arm'
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            'right_xyz', default_value='0.417 0 0',
            description='XYZ position of right arm'
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            'right_rpy', default_value='0 0 3.14159',
            description='RPY position of right arm'
        ))

# Initialize Arguments
    denso_robot_model = LaunchConfiguration('model')
    left_ip_address = LaunchConfiguration('left_ip_address')
    right_ip_address = LaunchConfiguration('right_ip_address')
    send_format = LaunchConfiguration('send_format')
    recv_format = LaunchConfiguration('recv_format')
    bcap_slave_control_cycle_msec = LaunchConfiguration('bcap_slave_control_cycle_msec')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    moveit_config_package = LaunchConfiguration('moveit_config_package')
    moveit_config_file = LaunchConfiguration('moveit_config_file')
    namespace = LaunchConfiguration('namespace')
    rviz = LaunchConfiguration('rviz')
    sim = LaunchConfiguration('sim')
    basic_camera = LaunchConfiguration('basic_camera')
    verbose = LaunchConfiguration('verbose')
    controllers_file = LaunchConfiguration('controllers_file')
    left_robot_controller = LaunchConfiguration('left_robot_controller')
    right_robot_controller = LaunchConfiguration('right_robot_controller')
    left_xyz = LaunchConfiguration('left_xyz')
    left_rpy = LaunchConfiguration('left_rpy')
    right_xyz = LaunchConfiguration('right_xyz')
    right_rpy = LaunchConfiguration('right_rpy')

    denso_robot_core_pkg = get_package_share_directory('denso_robot_core')

    denso_robot_control_parameters = {
        'denso_bcap_slave_control_cycle_msec': bcap_slave_control_cycle_msec,
        'denso_config_file': PathJoinSubstitution([denso_robot_core_pkg, 'config', 'config.xml'])}

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'urdf', description_file]),
            ' ',
            'left_ip_address:=', left_ip_address, ' ',
            'right_ip_address:=', right_ip_address, ' ',
            'model:=', denso_robot_model, ' ',
            'send_format:=', send_format, ' ',
            'recv_format:=', recv_format, ' ',
            'namespace:=', namespace, ' ',
            'verbose:=', verbose, ' ',
            'sim:=', sim, ' ',
            'basic_camera:=', basic_camera, ' ',
            'left_xyz:="', left_xyz, '" ',
            'left_rpy:="', left_rpy, '" ',
            'right_xyz:="', right_xyz, '" ',
            'right_rpy:="', right_rpy, '" '
        ])
    robot_description = {'robot_description': robot_description_content}

# --------- MoveIt Configuration ---------

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), 'srdf', moveit_config_file]),
            ' ',
            'model:=', denso_robot_model, ' ',
            'namespace:=', namespace, ' '
        ])
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}
    kinematics_yaml = load_yaml('denso_robot_moveit_config', 'config/dual/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # Planning Configuration
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization' \
                + ' default_planner_request_adapters/FixWorkspaceBounds' \
                + ' default_planner_request_adapters/FixStartStateBounds' \
                + ' default_planner_request_adapters/FixStartStateCollision' \
                + ' default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml('denso_robot_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    moveit_controllers = {
        'moveit_controller_manager': 'moveit_simple_controller_manager'\
            + '/MoveItSimpleControllerManager',
    }
    moveit_controllers_file = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_package), 'robots',
            denso_robot_model, 'config/dual/moveit_controllers.yaml'
        ])
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'planning_scene_monitor_options': {
            'name': 'planning_scene_monitor',
            'robot_description': 'robot_description',
            'joint_state_topic': '/joint_states',
            'attached_collision_object_topic': '/move_group/planning_scene_monitor',
            'publish_planning_scene_topic': '/move_group/publish_planning_scene',
            'monitored_planning_scene_topic': '/move_group/monitored_planning_scene',
            'wait_for_initial_state_timeout': 10.0,
        },
    }

    occupancy_map_monitor_parameters = {
        'sensors': ['3D_sensor'],
        '3D_sensor': {
            'sensor_plugin': '', #'~'
        },
    }
    robot_limits_file = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_package), 'robots',
            denso_robot_model, 'config/dual/joint_limits.yaml'
        ])

    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_limits_file,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            moveit_controllers_file,
            occupancy_map_monitor_parameters,
            planning_scene_monitor_parameters,
            {'use_sim_time': sim}
        ])

# --------- Robot Control Node (only if 'sim:=false') ---------
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(moveit_config_package), 'robots',
            denso_robot_model, 'config', 'dual', controllers_file
        ])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        condition=UnlessCondition(sim),
        parameters=[
            robot_description,
            robot_controllers,
            denso_robot_control_parameters
        ],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        })

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': sim}, robot_description]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['denso_joint_state_broadcaster', '--controller-manager', '/controller_manager'])

    left_robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[left_robot_controller, '-c', '/controller_manager'])
    
    right_robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[right_robot_controller, '-c', '/controller_manager'])

# --------- rviz with moveit configuration ---------
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), 'rviz', 'view_robot.rviz'])

    rviz_node = Node(
        package='rviz2',
        condition=IfCondition(rviz),
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics
        ])

    # Static TF
    left_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=[
            '--frame-id', 'world',
            '--child-frame-id', 'left_base_link'
        ])
    
    right_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=[
            '--frame-id', 'world',
            '--child-frame-id', 'right_base_link'
        ])

# --------- Gazebo Nodes (only if 'sim:=true') ---------
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = PathJoinSubstitution([
        FindPackageShare('denso_robot_gazebo'),
        'worlds',
        'empty_with_sensor_support.sdf'
    ])
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': ['-r', '-v4', ' ', world]}.items(), #'-r' == run simulation on start (without this flag gazebo not connect with ros2_controllers)
                                                                  #'-v 4' == verbose level 4 (max level of console output)
        condition=IfCondition(sim)
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        condition=IfCondition(sim),
        arguments=['-topic', 'robot_description', '-name', denso_robot_model],
        output='screen')

    #Node necessary to connect camera in gazebo to ROS topic
    ros_gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/left_basic_camera', '/right_basic_camera'], #camera topic name defined in the <topic> tag in the camera's .xacro file
        output='screen',
        condition=IfCondition(sim and basic_camera)
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml('denso_robot_moveit_config', 'config/moveit_servo.yaml')

    left_servo_params = {'moveit_servo': servo_yaml,
                    'moveit_servo.use_gazebo': sim,
                    'moveit_servo.move_group_name': 'left_arm',
                    'moveit_servo.planning_frame': 'world',
                    'moveit_servo.ee_frame_name': 'left_J6',
                    'moveit_servo.robot_link_command_frame': 'left_base_link',
                    'moveit_servo.command_out_topic': '/left_denso_joint_trajectory_controller/joint_trajectory'
    }

    left_servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='left_servo_node',
        parameters=[
            left_servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': sim}
        ],
        output='screen',
    )

    right_servo_params = {'moveit_servo': servo_yaml,
                    'moveit_servo.use_gazebo': sim,
                    'moveit_servo.move_group_name': 'right_arm',
                    'moveit_servo.planning_frame': 'world',
                    'moveit_servo.ee_frame_name': 'right_J6',
                    'moveit_servo.robot_link_command_frame': 'right_base_link',
                    'moveit_servo.command_out_topic': '/right_denso_joint_trajectory_controller/joint_trajectory'
    }

    right_servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='right_servo_node',
        parameters=[
            right_servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': sim}
        ],
        output='screen',
    )

    nodes_to_start = [
        control_node,
        left_robot_controller_spawner,
        right_robot_controller_spawner,
        move_group_node,
        rviz_node,
        left_static_tf,
        right_static_tf,
        gazebo,
        spawn_entity,
        ros_gz_image_bridge,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        left_servo_node,
        right_servo_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
