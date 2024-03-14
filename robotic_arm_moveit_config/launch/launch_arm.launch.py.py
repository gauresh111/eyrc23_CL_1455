from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

import launch_ros

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess


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
        parameters=[
            moveit_config.robot_description
        ],
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
