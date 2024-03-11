from moveit_configs_utils import MoveItConfigsBuilder
import os
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
    )
    moveit_config = MoveItConfigsBuilder(
        "robotic_arm", package_name="robotic_arm_moveit_config"
    ).to_moveit_configs()

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "robotic_arm", "-topic", "robot_description"],
        output="screen",
    )

    return LaunchDescription(
        [
            generate_demo_launch(moveit_config),
            # spawn_entity,
            # gazebo_launch,
        ]
    )
