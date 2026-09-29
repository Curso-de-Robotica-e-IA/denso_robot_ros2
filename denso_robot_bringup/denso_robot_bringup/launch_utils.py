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

from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

def control_node(robot_controllers_path, bcap_slave_control_cycle_msec, sim):

    denso_robot_control_parameters = {
        'denso_bcap_slave_control_cycle_msec': bcap_slave_control_cycle_msec,
        'denso_config_file': get_package_share_directory('denso_robot_core') 
            + '/config/config.xml'
    }

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        condition=UnlessCondition(sim),
        parameters=[
            robot_controllers_path,
            denso_robot_control_parameters
        ],
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        }
    )

    return control_node

def controller_spawner(controller, use_sim_time):

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            controller,
            '-c',
            '/controller_manager'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return controller_spawner

def move_group(move_config, use_sim_time):

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            move_config.to_dict(),
            {'use_sim_time': use_sim_time}
        ]
    )

    return move_group

def rviz(moveit_config, launch_rviz, use_sim_time):

    rviz_config_file = moveit_config.package_path / 'rviz' / 'view_robot.rviz'

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(launch_rviz),
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': use_sim_time}
        ]
    )

    return rviz

def static_tf(child_frame, use_sim_time):

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'static_transform_publisher_{child_frame}',
        output='log',
        arguments=[
            '--frame-id', 'world',
            '--child-frame-id', child_frame
        ],
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    return static_tf

def robot_state_publisher(robot_description, use_sim_time):
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    return robot_state_publisher

def moveit_servo(moveit_config, sim, arm=None):

    servo_params = (
        ParameterBuilder(moveit_config.package_path.name)
        .yaml(
            parameter_namespace='moveit_servo',
            file_path='config/moveit_servo.yaml'
        )
    )

    if arm == 'left':
        servo_params = (
            servo_params
            .parameter('moveit_servo.move_group_name', 'left_arm')
            .parameter('moveit_servo.command_out_topic', '/left_denso_joint_trajectory_controller/joint_trajectory')
        )

    elif arm == 'right':
        servo_params = (
            servo_params
            .parameter('moveit_servo.move_group_name', 'right_arm')
            .parameter('moveit_servo.command_out_topic', '/right_denso_joint_trajectory_controller/joint_trajectory')
        )

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name=f'{arm}_servo_node' if arm else None,
        parameters=[
            servo_params.to_dict(),
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': sim}
        ],
        output='screen',
    )

    return servo_node
