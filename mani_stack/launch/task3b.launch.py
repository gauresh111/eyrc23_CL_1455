import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

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
            'ros2 run mani_stack ',
        ]],
        shell=True
    )

    start_docking = ExecuteProcess(
        cmd=[[
            'ros2 run ebot_docking ebot_docking_boilerplate.py',
        ]],
        shell=True
    )

    start_navigation = ExecuteProcess(
        cmd=[[
            'ros2 run ebot_docking task2b.py',
        ]],
        shell=True
    )
 
    return LaunchDescription([
    start_perception,
     start_docking,
     start_navigation
    ])

