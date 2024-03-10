import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import (OnProcessStart, OnProcessExit)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )
    
    package_path = os.path.join(get_package_share_directory('robotic_arm'))

    xacro_file = os.path.join(package_path, 'urdf', 'robotic_arm.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robotic_arm', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        node_robot_state_publisher,
        spawn_entity
    ])