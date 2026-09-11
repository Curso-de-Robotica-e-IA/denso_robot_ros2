# Copyright (c) 2021 DENSO WAVE INCORPORATED
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: DENSO WAVE INCORPORATED

from pathlib import Path
from launch import LaunchDescription
from launch.conditions import IfCondition
from denso_robot_bringup import launch_utils
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument, 
    OpaqueFunction
)

def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration('model')
    ip_address = LaunchConfiguration('ip_address')
    send_format = LaunchConfiguration('send_format')
    recv_format = LaunchConfiguration('recv_format')
    bcap_slave_control_cycle_msec = LaunchConfiguration('bcap_slave_control_cycle_msec')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    moveit_config_package = LaunchConfiguration('moveit_config_package')
    moveit_config_file = LaunchConfiguration('moveit_config_file')
    namespace = LaunchConfiguration('namespace')
    launch_rviz = LaunchConfiguration('rviz')
    sim = LaunchConfiguration('sim')
    verbose = LaunchConfiguration('verbose')
    controllers_file = LaunchConfiguration('controllers_file')
    robot_controller = LaunchConfiguration('robot_controller')
    gz_cam = LaunchConfiguration('gz_cam')
    gz_world = LaunchConfiguration('gz_world')
    xyz = LaunchConfiguration('xyz')
    rpy = LaunchConfiguration('rpy')

    moveit_config = (
        MoveItConfigsBuilder('denso_robot')
        .robot_description(
            file_path=get_package_share_directory(description_package.perform(context))
            + f'/urdf/{description_file.perform(context)}',
            mappings={
                'ip_address': ip_address.perform(context),
                'model': model.perform(context),
                'send_format': send_format.perform(context),
                'recv_format': recv_format.perform(context),
                'namespace': namespace.perform(context),
                'verbose': verbose.perform(context),
                'sim': sim.perform(context),
                'gz_cam': gz_cam.perform(context),
                'xyz': xyz.perform(context),
                'rpy': rpy.perform(context)
            }
        )
        .robot_description_semantic(
            file_path=f'srdf/{moveit_config_file.perform(context)}',
            mappings={
                'model': model.perform(context),
                'namespace': namespace.perform(context)
            }
        )
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(
            file_path=f'robots/{model.perform(context)}/config/joint_limits.yaml'
        )
        .trajectory_execution(
            file_path=f'robots/{model.perform(context)}/config/moveit_controllers.yaml'
        )
        .planning_pipelines(pipelines=['ompl'])
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .to_moveit_configs()
    )

    robot_controllers = PathJoinSubstitution([
        FindPackageShare(moveit_config_package),
        'robots',
        model, 
        'config', 
        controllers_file
    ])

    controllers = [
        robot_controller.perform(context),
        'denso_joint_state_broadcaster'
    ]

    ###############################################################################################

    nodes_to_start = []

    nodes_to_start.append(
        launch_utils.control_node( 
            robot_controllers.perform(context),
            bcap_slave_control_cycle_msec,
            sim
        )
    )

    for controller in controllers:
        nodes_to_start.append(
            launch_utils.controller_spawner(controller, sim)
        )

    nodes_to_start.append(
        launch_utils.move_group(moveit_config, sim)
    )
    nodes_to_start.append(
        launch_utils.rviz(moveit_config, launch_rviz, sim)
    )
    nodes_to_start.append(
        launch_utils.static_tf('base_link', sim)
    )
    nodes_to_start.append(
        launch_utils.robot_state_publisher(moveit_config.robot_description, sim)
    )
    nodes_to_start.append(
        launch_utils.moveit_servo(moveit_config, sim)
    )

    ###############################################################################################

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            Path(
                get_package_share_directory('denso_robot_bringup')
            ) / 'launch' / 'gazebo.launch.py'
        ),
        launch_arguments={
            'gz_cam': gz_cam,
            'model': model,
            'camera_topics': '/gz_cam',
            'gz_world': gz_world,
        }.items(),
        condition=IfCondition(sim)
    )

    nodes_to_start.append(gazebo_launch)

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    # Denso specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'model',
            choices=['vs050', 'vs060'],
            description='Type/series of used denso robot.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'send_format',
            default_value='288',
            description='Data format for sending commands to the robot.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'recv_format', 
            default_value='292',
            description='Data format for receiving robot status.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'bcap_slave_control_cycle_msec',
            default_value='8.0',
            description='Control frequency.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ip_address',
            default_value='192.168.0.1',
            description='IP address by which the robot can be reached.'
        )
    )
    
    # Configuration arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='denso_robot_descriptions',
            description='Description package with robot URDF/XACRO files. Usually the argument' \
                + ' is not set, it enables use of a custom description.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='denso_robot.urdf.xacro',
            description='URDF/XACRO description file with the robot.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'moveit_config_package',
            default_value='denso_robot_moveit_config',
            description='MoveIt config package with robot SRDF/XACRO files. Usually the argument' \
                + ' is not set, it enables use of a custom moveit config.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'moveit_config_file',
            default_value='denso_robot.srdf.xacro',
            description='MoveIt SRDF/XACRO description file with the robot.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace', 
            default_value='',
            description='Prefix of the joint names, useful for' \
                + ' multi-robot setup. If changed than also joint names in the controllers' \
                + ' configuration have to be updated.'
            )
        )
    declared_arguments.append(
        DeclareLaunchArgument(
            'controllers_file',
            default_value='denso_robot_controllers.yaml',
            description='YAML file with the controllers configuration.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_controller',
            default_value='denso_joint_trajectory_controller',
            description='Robot controller to start.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz?'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'sim',
            default_value='true',
            description='Start robot with fake hardware mirroring command to its states.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Print out additional debug information.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gz_cam',
            default_value='false',
            description='Add gz_cam on end-effector'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gz_world',
            default_value='empty_with_sensor_support.sdf',
            description='Name of the Gazebo world file to be loaded.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'xyz',
            default_value='0 0 0',
            description='XYZ position of arm'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rpy',
            default_value='0 0 0',
            description='RPY position of arm'
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])