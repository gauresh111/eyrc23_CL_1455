from moveit_configs_utils import MoveItConfigsBuilder
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
import os
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from ament_index_python import get_package_share_directory

import launch_ros

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess


def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "robotic_arm", package_name="robotic_arm_moveit_config"
    ).to_moveit_configs()

    ld = LaunchDescription()

    # ld.add_action(
    #     launch_ros.actions.Node(
    #         package="gazebo_ros",
    #         name="spawn_entity",
    #         executable="spawn_entity.py",
    #         arguments=[
    #             "-entity",
    #             "robotic_arm",
    #             "-topic",
    #             "/robot_description",
    #             "-x",
    #             "0.0",
    #             "-y",
    #             "0.0",
    #             "-z",
    #             "0.0",
    #             "-Y",
    #             "0.0",
    #         ],
    #         output="screen",
    #     )
    # )

    # ld.add_action(
    #     ExecuteProcess(
    #         cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"],
    #         output="screen",
    #     )
    # )

    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            respawn=True,
            output="screen",
            parameters=[moveit_config.robot_description],
            # remappings=[('robot_description', 'robot_description_arm')]
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )
    # Servo node for realtime control
    moveit_config_folder_name = "robotic_arm_moveit_config"
    servo_config = (
        MoveItConfigsBuilder("robotic_arm")
        .robot_description(file_path="config/robotic_arm.urdf.xacro")
        .to_moveit_configs()
    )
    moveit_servo_file = get_package_file(
        moveit_config_folder_name, "config/robotic_arm_servo.yaml"
    )
    servo_yaml = load_yaml(moveit_servo_file)
    servo_params = {"moveit_servo": servo_yaml}
    ld.add_action(
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            parameters=[
                servo_params,
                servo_config.robot_description,
                servo_config.robot_description_semantic,
                servo_config.robot_description_kinematics,  # here is where kinematics plugin parameters are passed
            ],
        )
    )

    # Fake joint driver
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/spawn_controllers.launch.py")
            ),
        )
    )

    return ld
