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
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    OpaqueFunction
)

def launch_setup(context, *args, **kwargs):
    model = LaunchConfiguration('model')
    gz_cam = LaunchConfiguration('gz_cam')
    camera_topics = LaunchConfiguration('camera_topics')
    gz_world = LaunchConfiguration('gz_world')
    
    
    world = Path(get_package_share_directory('denso_robot_gazebo')) / 'worlds' / gz_world.perform(context)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            Path(get_package_share_directory('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py'
        ),
        launch_arguments={'gz_args': ['-r -v1 ', world]}.items()
        # '-r'   == run simulation on start (required for ros2_controllers to connect)
        # '-v1' == verbose level 1 (minimum console output)
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic',
            'robot_description',
            '-name',
            model
        ],
        output='screen'
    )

    ros_gz_clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Bridge Gazebo camera topics to ROS only when a camera is requested
    ros_gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        # camera_topics is a list of topic names defined in the camera .xacro file
        arguments=camera_topics.perform(context).split(),
        output='screen',
        condition=IfCondition(gz_cam)
    )

    return [gazebo, spawn_entity, ros_gz_clock_bridge, ros_gz_image_bridge]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'model',
            default_value='',
            description='Robot model name (used as the spawned entity name).'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gz_cam',
            default_value='false',
            description='Enable Gazebo-to-ROS camera image bridge.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_topics',
            default_value='/gz_cam',
            description='Space-separated list of camera topic names for the image bridge.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gz_world',
            default_value='empty_with_sensor_support.sdf',
            description='Name of the Gazebo world file to be loaded.'
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])