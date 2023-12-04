from launch.actions import ExecuteProcess
import os
import yaml
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)


from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml
def generate_launch_description():
    # start_perception = Node(
    # package='mani_stack',
    # executable='task1ab-perception',
    # )
    # start_docking = Node(
    # package='ebot_docking',
    # executable='ebot_docking_boilerplate',
    # )
    # start_navigation = Node(
    # package='ebot_docking',
    # executable='task2b',
    # )
    
    start_perception = ExecuteProcess(
        cmd=[[
            'ros2 run mani_stack perception.py ',
        ]],
        shell=True
    )

    start_docking = ExecuteProcess(
        cmd=[[
            'ros2 run ebot_docking ebot_docking_boilerplate.py',
        ]],
        shell=True
    )
    star_ServoManipulation = ExecuteProcess(
    cmd=[['ros2 run mani_stack servoManipulation_nav2.py']],
    shell=True
    )
    start_yaml_controller = ExecuteProcess(
        cmd=[[
            'ros2 run ebot_docking ebot_nav2_yaml.py',
        ]],
        shell=True
    )
 
    return LaunchDescription([
     start_perception,
     start_docking,
     start_yaml_controller,
     star_ServoManipulation
    ])

